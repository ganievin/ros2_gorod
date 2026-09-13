#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from city_interfaces.msg import GraphUpdate, TrackingInfo


# ---------- Таблицы направлений ----------
# Геометрия: A=(-a,0), B=(0,a), C=(a,0), D=(0,-a), E=(0,0).
# Курс = направление последней половины ребра-входа.
# straight/right/left — куда ведёт выезд из узла; None = такого выезда нет.

TURN_TABLE = {
    # --- узел A (соседи B, D, E) ---
    ('B', 'A'): {'straight': 'D', 'right': None, 'left': 'E'},
    ('D', 'A'): {'straight': 'B', 'right': 'E', 'left': None},
    ('E', 'A'): {'straight': None, 'right': 'B', 'left': 'D'},

    # --- узел B (соседи A, C, E) ---
    ('A', 'B'): {'straight': 'C', 'right': 'E', 'left': None},
    ('C', 'B'): {'straight': 'A', 'right': None, 'left': 'E'},
    ('E', 'B'): {'straight': None, 'right': 'C', 'left': 'A'},

    # --- узел C (соседи B, D, E) ---
    ('B', 'C'): {'straight': 'D', 'right': 'E', 'left': None},
    ('D', 'C'): {'straight': 'B', 'right': None, 'left': 'E'},
    ('E', 'C'): {'straight': None, 'right': 'D', 'left': 'B'},

    # --- узел D (соседи A, C, E) ---
    ('A', 'D'): {'straight': None, 'right': 'A', 'left': 'C'},
    ('C', 'D'): {'straight': 'A', 'right': 'E', 'left': None},
    ('E', 'D'): {'straight': None, 'right': 'A', 'left': 'C'},

    # --- центральный узел E (соседи A, B, C, D) ---
    ('A', 'E'): {'straight': 'C', 'right': 'D', 'left': 'B'},
    ('B', 'E'): {'straight': 'D', 'right': 'A', 'left': 'C'},
    ('C', 'E'): {'straight': 'A', 'right': 'B', 'left': 'D'},
    ('D', 'E'): {'straight': 'B', 'right': 'C', 'left': 'A'},
}


class Road:
    """Одно направленное ребро графа дорог."""
    def __init__(self, name):
        self.name = name
        self.is_visited = False
        self.forbidden_entry = []
        self.have_parking = False
        self.have_passengers = False


class SignHandlerNode(Node):
    def __init__(self):
        super().__init__('sign_handler_node')

        self.declare_parameter('tracking_topic', '/tracking_topic')
        self.declare_parameter('detected_sign_topic', '/detected_sign')
        self.declare_parameter('sign_is_close_topic', '/sign_is_close')
        self.declare_parameter('graph_updates_topic', '/graph_updates')
        self.declare_parameter('stop_duration', 3.0)
        self.declare_parameter('initial_position', 'DA')

        self.tracking_topic = self.get_parameter('tracking_topic').value
        self.detected_sign_topic = self.get_parameter('detected_sign_topic').value
        self.sign_is_close_topic = self.get_parameter('sign_is_close_topic').value
        self.graph_updates_topic = self.get_parameter('graph_updates_topic').value
        self.stop_duration = self.get_parameter('stop_duration').value
        self.initial_position = self.get_parameter('initial_position').value

        # Состояние
        self.current_node = None
        self.current_edge = None
        self.entry_edge = None
        self.last_sign = None
        self.sign_is_close = False
        self.is_stopped = False
        self.stop_timer = None
        self.applied_on_edge = set()
        self.applied_position_key = None   # (from, node) или (u, v) — для сброса applied_on_edge

        # Граф: 16 направленных рёбер
        self.edges = [
            ('A', 'B'), ('B', 'A'),
            ('B', 'C'), ('C', 'B'),
            ('C', 'D'), ('D', 'C'),
            ('D', 'A'), ('A', 'D'),
            ('A', 'E'), ('E', 'A'),
            ('B', 'E'), ('E', 'B'),
            ('C', 'E'), ('E', 'C'),
            ('D', 'E'), ('E', 'D'),
        ]
        self.roads = []
        self.road_map = {}
        for u, v in self.edges:
            name = u + v
            road = Road(name)
            self.roads.append(road)
            self.road_map[(u, v)] = road

        # Подписки
        self.create_subscription(
            TrackingInfo, self.tracking_topic, self.tracking_callback, 10)
        self.create_subscription(
            String, self.detected_sign_topic, self.sign_callback, 10)
        self.create_subscription(
            Bool, self.sign_is_close_topic, self.close_callback, 10)

        # Публикация
        self.graph_updates_pub = self.create_publisher(
            GraphUpdate, self.graph_updates_topic, 10)

        self.get_logger().info('Sign Handler Node started')
        self.get_logger().info(f'Stop duration: {self.stop_duration} seconds')
        self.get_logger().info(f'Initial position parameter: {self.initial_position!r}')

        self.apply_initial_position()

    # ---------- Вспомогательные ----------
    def get_road(self, u, v):
        return self.road_map.get((u, v))

    def get_neighbors(self, node):
        neighbors = []
        for u, v in self.edges:
            if u == node and v not in neighbors:
                neighbors.append(v)
            elif v == node and u not in neighbors:
                neighbors.append(u)
        return neighbors

    def publish_graph_update(self, road_name, attribute_name, values):
        msg = GraphUpdate()
        msg.road_name = road_name
        msg.attribute_name = attribute_name
        msg.values = list(values)
        self.graph_updates_pub.publish(msg)
        self.get_logger().info(
            f'Graph update: {road_name} {attribute_name} = {values}')

    def _set_position_key(self, key):
        """Сброс applied_on_edge при смене позиции."""
        if key != self.applied_position_key:
            self.applied_on_edge = set()
            self.applied_position_key = key

    def _current_position(self):
        """(from_node, node) или (None, None)."""
        if self.current_edge is not None:
            return self.current_edge
        if self.entry_edge is not None and self.current_node is not None:
            return self.entry_edge
        return (None, None)

    def _turn_dirs(self, from_node, node):
        key = (from_node, node)
        if key in TURN_TABLE:
            return TURN_TABLE[key]
        self.get_logger().warn(f'No turn table for from={from_node} to={node}')
        return {'straight': None, 'right': None, 'left': None}

    def apply_initial_position(self):
        pos = str(self.initial_position).strip()
        if not pos:
            return
        if len(pos) == 1 and pos in ['A', 'B', 'C', 'D', 'E']:
            self.current_node = pos
            self.current_edge = None
            self._set_position_key(('node', pos))
            self.get_logger().info(f'Initial position: node {pos}')
        elif len(pos) == 2 and pos[0] in 'ABCDE' and pos[1] in 'ABCDE':
            u, v = pos[0], pos[1]
            self.current_edge = (u, v)
            self.current_node = None
            self._set_position_key(('edge', u, v))
            self.get_logger().info(f'Initial position: edge {pos}')
            road = self.get_road(u, v)
            if road and not road.is_visited:
                road.is_visited = True
                self.publish_graph_update(road.name, 'is_visited', ['yes'])
        else:
            self.get_logger().warn(f'Invalid initial_position: {pos!r}')

    # ---------- Callback'и ----------
    def tracking_callback(self, msg: TrackingInfo):
        if self.is_stopped:
            return
        pt = msg.position_type.strip()

        if pt == 'node':
            node = msg.node_name.strip()
            entry = msg.entry_edge.strip()
            if node not in ['A', 'B', 'C', 'D', 'E']:
                self.get_logger().warn(f'Invalid node_name: {node!r}')
                return
            if entry and (len(entry) != 2 or entry[0] not in 'ABCDE' or entry[1] not in 'ABCDE'):
                self.get_logger().warn(f'Invalid entry_edge: {entry!r}')
                entry = ''
            if entry:
                from_node = entry[0]
                self._set_position_key(('node', node, from_node))
            else:
                self._set_position_key(('node', node, None))
            self.current_node = node
            self.current_edge = None
            self.entry_edge = (entry[0], entry[1]) if entry else None
            self.get_logger().info(
                f'Current node: {node} entry_edge={entry or "-"} heading={msg.heading_deg}')

        elif pt == 'edge':
            edge = msg.edge_name.strip()
            if len(edge) != 2 or edge[0] not in 'ABCDE' or edge[1] not in 'ABCDE':
                self.get_logger().warn(f'Invalid edge_name: {edge!r}')
                return
            u, v = edge[0], edge[1]
            self._set_position_key(('edge', u, v))
            self.current_edge = (u, v)
            self.current_node = None
            self.entry_edge = None
            self.get_logger().info(
                f'Current edge: {edge} progress={msg.progress:.2f} heading={msg.heading_deg}')
            road = self.get_road(u, v)
            if road and not road.is_visited:
                road.is_visited = True
                self.publish_graph_update(road.name, 'is_visited', ['yes'])

        elif pt == 'unknown':
            pass
        else:
            self.get_logger().warn(f'Unknown position_type: {pt!r}')

    def sign_callback(self, msg):
        self.last_sign = msg.data
        self.get_logger().info(f'Last sign: {self.last_sign}')

    def close_callback(self, msg):
        self.sign_is_close = msg.data
        if not self.sign_is_close or self.last_sign is None:
            return
        from_node, node = self._current_position()
        if from_node is None:
            return
        self.apply_sign_restriction(from_node, node, self.last_sign)

    # ---------- Основная логика ----------
    def apply_sign_restriction(self, from_node, node, sign):
        if sign in self.applied_on_edge:
            return
        self.applied_on_edge.add(sign)

        current_road = self.get_road(from_node, node)
        if current_road is None:
            self.get_logger().warn(f'Current road not found for {from_node}-{node}')
            return

        all_exits = self.get_neighbors(node)
        turn = self._turn_dirs(from_node, node)

        # --- Знаки «Остановка»/«Парковка»: отметка на текущем ребре ---
        if sign == 'parkovka':
            if not current_road.have_parking:
                current_road.have_parking = True
                self.publish_graph_update(current_road.name, 'have_parking', ['yes'])
            return
        if sign == 'ostanovka':
            if not current_road.have_passengers:
                current_road.have_passengers = True
                self.publish_graph_update(current_road.name, 'have_passengers', ['yes'])
            return
        if sign == 'opasnost':
            self.stop_robot()
            return

        # --- Знаки направления ---
        if sign == 'pravo':
            allowed = turn['right']
            desc = 'right'
        elif sign == 'levo':
            allowed = turn['left']
            desc = 'left'
        elif sign == 'pryamo':
            allowed = turn['straight']
            desc = 'straight'
        elif sign == 'nepravo':
            forbidden_only = turn['right']
            if forbidden_only is None:
                self.get_logger().warn(
                    f'Node {node} (from {from_node}): no right to forbid')
                return
            self._forbid(node, from_node, current_road, [forbidden_only])
            self.get_logger().info(
                f'Node {node} (from {from_node}): right to {forbidden_only} forbidden')
            return
        elif sign == 'nelevo':
            forbidden_only = turn['left']
            if forbidden_only is None:
                self.get_logger().warn(
                    f'Node {node} (from {from_node}): no left to forbid')
                return
            self._forbid(node, from_node, current_road, [forbidden_only])
            self.get_logger().info(
                f'Node {node} (from {from_node}): left to {forbidden_only} forbidden')
            return
        else:
            self.get_logger().warn(f'Unknown sign: {sign!r}')
            self.stop_robot()
            return

        if allowed is None:
            self.get_logger().warn(
                f'Node {node} (from {from_node}): no {desc} direction')
            return

        # Запрещаем все выезды из узла, кроме разрешённого.
        # all_exits включает from_node — так запрещается и U-turn.
        forbidden = [x for x in all_exits if x != allowed]
        self._forbid(node, from_node, current_road, forbidden)
        self.get_logger().info(
            f'Node {node} (from {from_node}): only {desc} to {allowed}')

    def _forbid(self, node, from_node, current_road, targets):
        for d in targets:
            target_road = self.get_road(node, d)
            if target_road is None:
                continue
            if current_road.name in target_road.forbidden_entry:
                continue
            target_road.forbidden_entry.append(current_road.name)
            self.publish_graph_update(
                target_road.name, 'forbidden_entry', [current_road.name])

    def stop_robot(self):
        if self.is_stopped:
            return
        self.is_stopped = True
        self.get_logger().info(f'Robot stopped for {self.stop_duration} seconds')
        if self.stop_timer is not None:
            self.stop_timer.cancel()
        self.stop_timer = self.create_timer(self.stop_duration, self.resume_robot)

    def resume_robot(self):
        self.is_stopped = False
        self.stop_timer = None
        self.get_logger().info('Robot resumed')


def main(args=None):
    rclpy.init(args=args)
    node = SignHandlerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
