#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tracker_node — определяет положение робота в графе дорог.

Ключевое: каждое L-ребро имеет ДВА направления (первая и вторая половина).
Направления заданы явно в EDGE_DIRECTIONS — знак ±90 НЕ применяется.

СК: +X вправо, +Y вверх. Углы в градусах, CCW от +X.
"""

import math
from typing import Dict, Optional, Tuple, List

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy,
)

from nav_msgs.msg import Odometry
from city_interfaces.msg import Graph, TrackingInfo


# ============================================================
#                  ГЕОМЕТРИЯ ПОЛИГОНА
# ============================================================

NODE_BBOX: Dict[str, Tuple[float, float, float, float]] = {
    'A': (-0.6, 0.2, -1.8, -1.0),
    'B': ( 1.0, 1.8, -0.2,  0.6),
    'C': ( 2.6, 3.4, -1.8, -1.0),
    'D': ( 1.0, 1.8, -3.4, -2.6),
    'E': ( 1.0, 1.8, -1.8, -1.0),
}

EDGES_BBOX: Dict[str, Tuple[float, float, float, float]] = {
    'AB': (-0.6, 1.0, -1.0, 0.6),
    'BA': (-0.6, 1.0, -1.0, 0.6),
    'BC': ( 1.8, 3.4, -1.0, 0.6),
    'CB': ( 1.8, 3.4, -1.0, 0.6),
    'CD': ( 1.8, 3.4, -3.4, -1.8),
    'DC': ( 1.8, 3.4, -3.4, -1.8),
    'DA': (-0.6, 1.0, -3.4, -1.8),
    'AD': (-0.6, 1.0, -3.4, -1.8),
    'AE': ( 0.2, 1.0, -1.8, -1.0),
    'EA': ( 0.2, 1.0, -1.8, -1.0),
    'BE': ( 1.0, 1.8, -1.0, -0.2),
    'EB': ( 1.0, 1.8, -1.0, -0.2),
    'CE': ( 1.8, 2.6, -1.8, -1.0),
    'EC': ( 1.8, 2.6, -1.8, -1.0),
    'DE': ( 1.0, 1.8, -2.6, -1.8),
    'ED': ( 1.0, 1.8, -2.6, -1.8),
}

# Явные направления каждой половины ребра (град CCW от +X).
# Для L-рёбер — два угла (первая и вторая половина).
# Для прямых — один.
EDGE_DIRECTIONS: Dict[str, List[float]] = {
    'AB': [90.0, 0.0],      # вверх, потом вправо
    'BA': [180.0, 270.0],   # влево, потом вниз
    'BC': [0.0, 270.0],     # вправо, потом вниз
    'CB': [90.0, 180.0],    # вверх, потом влево
    'CD': [270.0, 180.0],   # вниз, потом влево
    'DC': [0.0, 90.0],      # вправо, потом вверх
    'DA': [180.0, 90.0],    # влево, потом вверх
    'AD': [270.0, 0.0],     # вниз, потом вправо
    'AE': [0.0],
    'EA': [180.0],
    'BE': [270.0],
    'EB': [90.0],
    'CE': [180.0],
    'EC': [0.0],
    'DE': [90.0],
    'ED': [270.0],
}

HEADING_TOL_DEG = 25.0
BOUNDARY_MARGIN = 0.03


# ============================================================
#                       УТИЛИТЫ
# ============================================================

def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff_deg(a_deg: float, b_deg: float) -> float:
    return (a_deg - b_deg + 180.0) % 360.0 - 180.0


def point_in_bbox(x: float, y: float,
                  bbox: Tuple[float, float, float, float],
                  margin: float = 0.0) -> bool:
    x_min, x_max, y_min, y_max = bbox
    return (x_min - margin <= x <= x_max + margin and
            y_min - margin <= y <= y_max + margin)


# ============================================================
#                        НОДА
# ============================================================

class TrackerNode(Node):

    def __init__(self):
        super().__init__('tracker_node')

        self.declare_parameter('graph_topic', '/graph')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('output_topic', '/tracking_topic')
        self.declare_parameter('heading_tol_deg', HEADING_TOL_DEG)
        self.declare_parameter('publish_on_change_only', True)
        self.declare_parameter('debug_edge_match', True)

        self.heading_tol = float(self.get_parameter('heading_tol_deg').value)
        self.publish_on_change_only = bool(
            self.get_parameter('publish_on_change_only').value)
        self.debug_edge_match = bool(
            self.get_parameter('debug_edge_match').value)

        # Ориентации из графа (необязательны — есть явная таблица)
        self.graph_orientation: Dict[str, float] = {}

        self.current_node: Optional[str] = None
        self.current_edge: Optional[str] = None
        self.entry_edge: Optional[str] = None
        self.last_published_key: Optional[tuple] = None

        graph_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1,
        )
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10,
        )

        self.create_subscription(
            Graph, self.get_parameter('graph_topic').value,
            self.on_graph, graph_qos)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self.on_odom, odom_qos)
        self.pub = self.create_publisher(
            TrackingInfo, self.get_parameter('output_topic').value, 10)

        self.get_logger().info(
            f'tracker_node запущен | tol={self.heading_tol}° | '
            f'debug={self.debug_edge_match}')

    # ---------------------------------------------------------
    def on_graph(self, msg: Graph):
        # Читаем на случай, если понадобится — но в логике НЕ используем,
        # потому что явная таблица EDGE_DIRECTIONS надёжнее.
        self.graph_orientation = {
            r.name: float(r.orientation) for r in msg.roads
        }
        self.get_logger().info(
            f'Граф обновлён: {len(msg.roads)} рёбер '
            f'(ориентации для справки; классификация — по EDGE_DIRECTIONS)')

    # ---------------------------------------------------------
    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        yaw_deg = math.degrees(yaw_from_quaternion(
            msg.pose.pose.orientation))

        self.get_logger().info(
            f'[STATE] x={p.x:+.2f} y={p.y:+.2f} yaw={yaw_deg:+.0f}° | '
            f'cur_node={self.current_node or "-"} '
            f'cur_edge={self.current_edge or "-"} '
            f'entry={self.entry_edge or "-"}')

        info = TrackingInfo()
        info.heading_deg = int(round(yaw_deg))
        info.node_name = ''
        info.edge_name = ''
        info.entry_edge = ''
        info.progress = 0.0

        node_hit = self._match_node(p.x, p.y)
        if node_hit is not None:
            info.position_type = 'node'
            info.node_name = node_hit
            info.entry_edge = self.entry_edge or ''
            self._update_state('node', node_hit, None)
            self._publish(info, ('node', node_hit, self.entry_edge))
            return

        edge_hit, progress = self._match_edge(p.x, p.y, yaw_deg)
        if edge_hit is not None:
            info.position_type = 'edge'
            info.edge_name = edge_hit
            info.progress = float(progress)
            self._update_state('edge', None, edge_hit)
            self._publish(info, ('edge', edge_hit))
            return

        info.position_type = 'unknown'
        self._publish(info, ('unknown',))

    # ---------------------------------------------------------
    def _match_node(self, x: float, y: float) -> Optional[str]:
        for name, bbox in NODE_BBOX.items():
            if point_in_bbox(x, y, bbox):
                return name
        return None

    # ---------------------------------------------------------
    def _match_edge(self, x: float, y: float,
                    yaw_deg: float) -> Tuple[Optional[str], float]:
        """
        Классификация ребра:
          1) точка должна лежать в bbox ребра;
          2) курс должен совпасть с одним из EDGE_DIRECTIONS[name]
             в пределах heading_tol;
          3) из прошедших выбирается минимальная угловая ошибка.
        """
        best_name = None
        best_err = None
        best_bbox = None
        m = BOUNDARY_MARGIN

        bbox_candidates = []
        angle_candidates = []

        for name, bbox in EDGES_BBOX.items():
            if not point_in_bbox(x, y, bbox, m):
                continue
            bbox_candidates.append(name)

            dirs = EDGE_DIRECTIONS.get(name)
            if not dirs:
                continue

            err = min(abs(angle_diff_deg(yaw_deg, d)) for d in dirs)
            if err > self.heading_tol:
                continue
            angle_candidates.append((name, err))

            if best_err is None or err < best_err:
                best_name = name
                best_err = err
                best_bbox = bbox

        if self.debug_edge_match and bbox_candidates:
            bbox_str = ', '.join(bbox_candidates)
            angle_str = ', '.join(
                f'{n}(err={e:.1f}°)' for n, e in angle_candidates) or '—'
            self.get_logger().info(
                f'  bbox: [{bbox_str}] | '
                f'курс {yaw_deg:.0f}° прошли: [{angle_str}] | '
                f'выбрано: {best_name or "—"}')

        progress = 0.5
        if best_name is not None and best_bbox is not None:
            progress = self._estimate_progress(best_name, x, y, best_bbox)

        return best_name, progress

    # ---------------------------------------------------------
    def _estimate_progress(self, name: str, x: float, y: float,
                            bbox: Tuple[float, float, float, float]) -> float:
        """Грубая оценка прогресса по bbox."""
        x_min, x_max, y_min, y_max = bbox

        if name in ('AE', 'EA', 'CE', 'EC'):
            if x_max - x_min < 1e-6:
                return 0.5
            return max(0.0, min(1.0, (x - x_min) / (x_max - x_min)))
        if name in ('BE', 'EB', 'DE', 'ED'):
            if y_max - y_min < 1e-6:
                return 0.5
            return max(0.0, min(1.0, (y - y_min) / (y_max - y_min)))

        # L-образные — манхэттенское приближение от стартового угла
        side = max(x_max - x_min, y_max - y_min)
        if side < 1e-6:
            return 0.5
        src = {
            'AB': (x_min, y_min), 'BA': (x_max, y_max),
            'BC': (x_min, y_min), 'CB': (x_max, y_max),
            'CD': (x_min, y_max), 'DC': (x_max, y_min),
            'DA': (x_max, y_max), 'AD': (x_min, y_min),
        }.get(name, (x_min, y_min))
        dx = x - src[0]
        dy = y - src[1]
        return max(0.0, min(1.0, (abs(dx) + abs(dy)) / (2 * side)))

    # ---------------------------------------------------------
    def _update_state(self, kind: str,
                      node: Optional[str], edge: Optional[str]):
        if kind == 'node':
            if self.current_node != node:
                self.entry_edge = self.current_edge
            self.current_node = node
            self.current_edge = None
        elif kind == 'edge':
            if self.current_edge != edge:
                self.entry_edge = None
            self.current_edge = edge
            self.current_node = None

    # ---------------------------------------------------------
    def _publish(self, info: TrackingInfo, key: tuple):
        if self.publish_on_change_only and key == self.last_published_key:
            return
        self.pub.publish(info)
        if key != self.last_published_key:
            self.get_logger().info(
                f'Робот: type={info.position_type} '
                f'edge={info.edge_name or "-"} '
                f'node={info.node_name or "-"} '
                f'entry={info.entry_edge or "-"} '
                f'progress={info.progress:.2f} '
                f'heading={info.heading_deg}')
        self.last_published_key = key


# ============================================================

def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
