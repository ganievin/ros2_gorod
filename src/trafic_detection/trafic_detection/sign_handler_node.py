#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from city_interfaces.msg import GraphUpdate, TrackingInfo


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
        self.current_node = None       # 'A'..'E' при position_type == 'node'
        self.current_edge = None       # (u, v) при position_type == 'edge'
        self.entry_edge = None         # (u, v) — ребро, с которого приехали в узел
        self.last_sign = None
        self.sign_is_close = False
        self.is_stopped = False
        self.stop_timer = None
        self.applied_on_edge = set()

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
        self.sub_tracking = self.create_subscription(
            TrackingInfo, self.tracking_topic, self.tracking_callback, 10)
        self.sub_sign = self.create_subscription(
            String, self.detected_sign_topic, self.sign_callback, 10)
        self.sub_close = self.create_subscription(
            Bool, self.sign_is_close_topic, self.close_callback, 10)

        # Публикация
        self.graph_updates_pub = self.create_publisher(
            GraphUpdate, self.graph_updates_topic, 10)

        self.get_logger().info('Sign Handler Node started')
        self.get_logger().info(f'Stop duration: {self.stop_duration} seconds')
        self.get_logger().info(f'Initial position parameter: {self.initial_position!r}')

        self.apply_initial_position()

    # ---------- Вспомогательные методы ----------
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

    def _is_new_journey(self, u, v):
        """True, если (u, v) — новое ребро относительно текущего положения."""
        if self.current_edge is not None:
            return (u, v) != self.current_edge
        if self.entry_edge is not None:
            return (u, v) != self.entry_edge
        return True

    def _current_position(self):
        """Возвращает (from_node, target_node) или (None, None)."""
        if self.current_edge is not None:
            return self.current_edge
        if self.entry_edge is not None and self.current_node is not None:
            return self.entry_edge
        return (None, None)

    def apply_initial_position(self):
        pos = str(self.initial_position).strip()

        if not pos:
            self.get_logger().info('Initial position not set, starting empty')
            return

        if len(pos) == 1 and pos in ['A', 'B', 'C', 'D', 'E']:
            self.current_node = pos
            self.current_edge = None
            self.get_logger().info(f'Initial position: node {pos}')

        elif len(pos) == 2 and pos[0] in 'ABCDE' and pos[1] in 'ABCDE':
            u, v = pos[0], pos[1]
            self.current_edge = (u, v)
            self.current_node = None
            self.get_logger().info(f'Initial position: edge {pos}')

            road = self.get_road(u, v)
            if road and not road.is_visited:
                road.is_visited = True
                self.publish_graph_update(road.name, 'is_visited', ['yes'])

        else:
            self.get_logger().warn(
                f'Invalid initial_position: {pos!r} (expected "A" or "AB")')

    # ---------- Callback'и ----------
    def tracking_callback(self, msg: TrackingInfo):
        if self.is_stopped:
            return

        pt = msg.position_type.strip()

        if pt == 'node':
            node = msg.node_name.strip()
            entry = msg.entry_edge.strip()

            if node not in ['A', 'B', 'C', 'D', 'E']:
                self.get_logger().warn(
                    f'Invalid node_name in TrackingInfo: {node!r}')
                return

            if entry and (len(entry) != 2
                          or entry[0] not in 'ABCDE'
                          or entry[1] not in 'ABCDE'):
                self.get_logger().warn(
                    f'Invalid entry_edge in TrackingInfo: {entry!r}')
                entry = ''

            self.current_node = node
            self.current_edge = None
            self.entry_edge = (entry[0], entry[1]) if entry else None

            self.get_logger().info(
                f'Current node: {node} entry_edge={entry or "-"} '
                f'(heading={msg.heading_deg})')

        elif pt == 'edge':
            edge = msg.edge_name.strip()

            if (len(edge) != 2
                    or edge[0] not in 'ABCDE'
                    or edge[1] not in 'ABCDE'):
                self.get_logger().warn(
                    f'Invalid edge_name in TrackingInfo: {edge!r}')
                return

            u, v = edge[0], edge[1]
            if self._is_new_journey(u, v):
                self.applied_on_edge = set()

            self.current_edge = (u, v)
            self.current_node = None
            self.entry_edge = None

            self.get_logger().info(
                f'Current edge: {edge} progress={msg.progress:.2f} '
                f'heading={msg.heading_deg}')

            road = self.get_road(u, v)
            if road and not road.is_visited:
                road.is_visited = True
                self.publish_graph_update(road.name, 'is_visited', ['yes'])

        elif pt == 'unknown':
            self.get_logger().debug('Tracking position_type: unknown')

        else:
            self.get_logger().warn(
                f'Unknown position_type in TrackingInfo: {pt!r} '
                f'(expected "node" | "edge" | "unknown")')

    def sign_callback(self, msg):
        self.last_sign = msg.data
        self.get_logger().info(f'Last sign: {self.last_sign}')

    def close_callback(self, msg):
        self.sign_is_close = msg.data

        if not self.sign_is_close or self.last_sign is None:
            return

        from_node, _ = self._current_position()
        if from_node is None:
            return

        self.apply_sign_restriction(self.last_sign)

    # ---------- Основная логика ----------
    def apply_sign_restriction(self, sign):
        from_node, node = self._current_position()
        if from_node is None:
            self.get_logger().warn(
                'No current position, cannot apply sign restriction')
            return

        if sign in self.applied_on_edge:
            return
        self.applied_on_edge.add(sign)

        current_road = self.get_road(from_node, node)
        if current_road is None:
            self.get_logger().warn(
                f'Current road not found for {from_node}-{node}')
            return

        neighbors = self.get_neighbors(node)
        possible_directions = [n for n in neighbors if n != from_node]

        # ---- ПРЯМО ----
        if sign == 'pryamo':
            if node in ['A', 'B', 'C', 'D']:
                perim_neighbors = {
                    'A': ['B', 'D'],
                    'B': ['A', 'C'],
                    'C': ['B', 'D'],
                    'D': ['A', 'C']
                }
                if from_node in perim_neighbors[node]:
                    straight = [x for x in perim_neighbors[node] if x != from_node][0]
                else:
                    straight = perim_neighbors[node][0]
            else:
                opposite_map = {'A': 'C', 'B': 'D', 'C': 'A', 'D': 'B'}
                if from_node in opposite_map:
                    straight = opposite_map[from_node]
                else:
                    self.get_logger().warn(
                        'Node E: unknown entry direction, cannot determine straight')
                    return

            if straight in possible_directions:
                for d in possible_directions:
                    if d != straight:
                        target_road = self.get_road(node, d)
                        if (target_road
                                and current_road.name not in target_road.forbidden_entry):
                            target_road.forbidden_entry.append(current_road.name)
                            self.publish_graph_update(
                                target_road.name, 'forbidden_entry',
                                [current_road.name])
                self.get_logger().info(
                    f'Node {node}: only straight to {straight} allowed')
            else:
                self.get_logger().warn(
                    f'Straight direction {straight} not available from {node}')

        # ---- НАПРАВО ----
        elif sign == 'pravo':
            right_map = {'A': 'B', 'B': 'C', 'C': 'D', 'D': 'A', 'E': 'A'}
            right = right_map.get(node)
            if right in possible_directions:
                for d in possible_directions:
                    if d != right:
                        target_road = self.get_road(node, d)
                        if (target_road
                                and current_road.name not in target_road.forbidden_entry):
                            target_road.forbidden_entry.append(current_road.name)
                            self.publish_graph_update(
                                target_road.name, 'forbidden_entry',
                                [current_road.name])
                self.get_logger().info(
                    f'Node {node}: only right to {right} allowed')
            else:
                self.get_logger().warn(
                    f'Right direction {right} not available from {node}')

        # ---- НАЛЕВО ----
        elif sign == 'levo':
            left_map = {'A': 'D', 'B': 'A', 'C': 'B', 'D': 'C', 'E': 'D'}
            left = left_map.get(node)
            if left in possible_directions:
                for d in possible_directions:
                    if d != left:
                        target_road = self.get_road(node, d)
                        if (target_road
                                and current_road.name not in target_road.forbidden_entry):
                            target_road.forbidden_entry.append(current_road.name)
                            self.publish_graph_update(
                                target_road.name, 'forbidden_entry',
                                [current_road.name])
                self.get_logger().info(
                    f'Node {node}: only left to {left} allowed')
            else:
                self.get_logger().warn(
                    f'Left direction {left} not available from {node}')

        # ---- НЕ НАПРАВО ----
        elif sign == 'nepravo':
            right_map = {'A': 'B', 'B': 'C', 'C': 'D', 'D': 'A', 'E': 'A'}
            right = right_map.get(node)
            if right in possible_directions:
                target_road = self.get_road(node, right)
                if (target_road
                        and current_road.name not in target_road.forbidden_entry):
                    target_road.forbidden_entry.append(current_road.name)
                    self.publish_graph_update(
                        target_road.name, 'forbidden_entry',
                        [current_road.name])
                self.get_logger().info(
                    f'Node {node}: right turn to {right} forbidden')

        # ---- НЕ НАЛЕВО ----
        elif sign == 'nelevo':
            left_map = {'A': 'D', 'B': 'A', 'C': 'B', 'D': 'C', 'E': 'D'}
            left = left_map.get(node)
            if left in possible_directions:
                target_road = self.get_road(node, left)
                if (target_road
                        and current_road.name not in target_road.forbidden_entry):
                    target_road.forbidden_entry.append(current_road.name)
                    self.publish_graph_update(
                        target_road.name, 'forbidden_entry',
                        [current_road.name])
                self.get_logger().info(
                    f'Node {node}: left turn to {left} forbidden')

        # ---- ПАРКОВКА ----
        elif sign == 'parkovka':
            if not current_road.have_parking:
                current_road.have_parking = True
                self.publish_graph_update(
                    current_road.name, 'have_parking', ['yes'])

        # ---- ОСТАНОВКА ----
        elif sign == 'ostanovka':
            if not current_road.have_passengers:
                current_road.have_passengers = True
                self.publish_graph_update(
                    current_road.name, 'have_passengers', ['yes'])

        # ---- ОПАСНОСТЬ / НЕИЗВЕСТНЫЙ ----
        elif sign == 'opasnost':
            self.stop_robot()
        else:
            self.stop_robot()

    def stop_robot(self):
        if self.is_stopped:
            return
        self.is_stopped = True
        self.get_logger().info(
            f'Robot stopped for {self.stop_duration} seconds')
        if self.stop_timer is not None:
            self.stop_timer.cancel()
        self.stop_timer = self.create_timer(
            self.stop_duration, self.resume_robot)

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
