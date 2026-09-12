#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from city_interfaces.msg import GraphUpdate


class Road:
    """Класс, описывающий участок дороги (ребро графа)."""
    def __init__(self, name):
        self.name = name
        self.is_visited = False
        self.forbidden_entry = []
        self.have_parking = False
        self.have_passengers = False


class SignHandlerNode(Node):
    def __init__(self):
        super().__init__('sign_handler_node')

        # Параметры
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
        self.last_sign = None
        self.sign_is_close = False
        self.is_stopped = False
        self.stop_timer = None

        # Граф
        self.edges = [
            ('A', 'B'), ('B', 'C'), ('C', 'D'), ('D', 'A'),
            ('A', 'E'), ('B', 'E'), ('C', 'E'), ('D', 'E')
        ]
        self.roads = []
        self.road_map = {}
        for u, v in self.edges:
            name = u + v
            road = Road(name)
            self.roads.append(road)
            self.road_map[frozenset([u, v])] = road

        # Подписки
        self.sub_tracking = self.create_subscription(
            String, self.tracking_topic, self.tracking_callback, 10)
        self.sub_sign = self.create_subscription(
            String, self.detected_sign_topic, self.sign_callback, 10)
        self.sub_close = self.create_subscription(
            Bool, self.sign_is_close_topic, self.close_callback, 10)

        # Публикация — теперь GraphUpdate
        self.graph_updates_pub = self.create_publisher(
            GraphUpdate, self.graph_updates_topic, 10)

        self.get_logger().info('Sign Handler Node started')
        self.get_logger().info(f'Stop duration: {self.stop_duration} seconds')
        self.get_logger().info(f'Initial position parameter: {self.initial_position!r}')

        # Инициализация начального положения робота
        self.apply_initial_position()

    # ---------- Вспомогательные методы ----------
    def get_road(self, u, v):
        return self.road_map.get(frozenset([u, v]))

    def get_neighbors(self, node):
        neighbors = []
        for u, v in self.edges:
            if u == node:
                neighbors.append(v)
            elif v == node:
                neighbors.append(u)
        return neighbors

    def publish_graph_update(self, road_name, attribute_name, values):
        """
        Публикует одно изменение атрибута ребра в виде city_interfaces/GraphUpdate.

        :param road_name: str, например 'BD'
        :param attribute_name: 'is_visited' | 'forbidden_entry' | 'have_parking' | 'have_passengers'
        :param values: list[str], например ['AB'] или ['yes']
        """
        msg = GraphUpdate()
        msg.road_name = road_name
        msg.attribute_name = attribute_name
        msg.values = list(values)
        self.graph_updates_pub.publish(msg)
        self.get_logger().info(
            f'Graph update: {road_name} {attribute_name} = {values}')

    def apply_initial_position(self):
        """Устанавливает начальное положение робота из параметра initial_position."""
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
            self.get_logger().warn(f'Invalid initial_position: {pos!r} (expected "A" or "AB")')

    # ---------- Callback'и ----------
    def tracking_callback(self, msg):
        data = msg.data.strip()

        if self.is_stopped:
            return

        # Одна буква = робот на узле
        if len(data) == 1 and data in ['A', 'B', 'C', 'D', 'E']:
            self.current_node = data
            self.current_edge = None
            self.get_logger().info(f'Current node: {data}')

        # Две буквы = робот на ребре
        elif len(data) == 2 and data[0] in 'ABCDE' and data[1] in 'ABCDE':
            u, v = data[0], data[1]
            self.current_edge = (u, v)
            self.current_node = None
            self.get_logger().info(f'Current edge: {data}')

            road = self.get_road(u, v)
            if road and not road.is_visited:
                road.is_visited = True
                self.publish_graph_update(road.name, 'is_visited', ['yes'])

        else:
            self.get_logger().warn(f'Unknown tracking format: {data!r}')

    def sign_callback(self, msg):
        self.last_sign = msg.data
        self.get_logger().info(f'Last sign: {self.last_sign}')

    def close_callback(self, msg):
        self.sign_is_close = msg.data
        if self.sign_is_close and self.last_sign is not None and self.current_node is not None:
            self.apply_sign_restriction(self.current_node, self.last_sign)

    # ---------- Основная логика ----------
    def apply_sign_restriction(self, node, sign):
        if self.current_edge is None:
            self.get_logger().warn('No current edge, cannot apply sign restriction')
            return

        from_node = None
        if self.current_edge[1] == node:
            from_node = self.current_edge[0]
        elif self.current_edge[0] == node:
            from_node = self.current_edge[1]

        current_road = self.get_road(from_node, node)
        if current_road is None:
            self.get_logger().warn(f'Current road not found for {from_node}-{node}')
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
                    self.get_logger().warn('Node E: unknown entry direction, cannot determine straight')
                    return

            if straight in possible_directions:
                for d in possible_directions:
                    if d != straight:
                        target_road = self.get_road(node, d)
                        if target_road and current_road not in target_road.forbidden_entry:
                            target_road.forbidden_entry.append(current_road)
                            self.publish_graph_update(
                                target_road.name, 'forbidden_entry', [current_road.name])
                self.get_logger().info(f'Node {node}: only straight to {straight} allowed')
            else:
                self.get_logger().warn(f'Straight direction {straight} not available from {node}')

        # ---- НАПРАВО ----
        elif sign == 'pravo':
            right_map = {'A': 'B', 'B': 'C', 'C': 'D', 'D': 'A', 'E': 'A'}
            right = right_map.get(node)
            if right in possible_directions:
                for d in possible_directions:
                    if d != right:
                        target_road = self.get_road(node, d)
                        if target_road and current_road not in target_road.forbidden_entry:
                            target_road.forbidden_entry.append(current_road)
                            self.publish_graph_update(
                                target_road.name, 'forbidden_entry', [current_road.name])
                self.get_logger().info(f'Node {node}: only right to {right} allowed')
            else:
                self.get_logger().warn(f'Right direction {right} not available from {node}')

        # ---- НАЛЕВО ----
        elif sign == 'levo':
            left_map = {'A': 'D', 'B': 'A', 'C': 'B', 'D': 'C', 'E': 'D'}
            left = left_map.get(node)
            if left in possible_directions:
                for d in possible_directions:
                    if d != left:
                        target_road = self.get_road(node, d)
                        if target_road and current_road not in target_road.forbidden_entry:
                            target_road.forbidden_entry.append(current_road)
                            self.publish_graph_update(
                                target_road.name, 'forbidden_entry', [current_road.name])
                self.get_logger().info(f'Node {node}: only left to {left} allowed')
            else:
                self.get_logger().warn(f'Left direction {left} not available from {node}')

        # ---- НЕ НАПРАВО ----
        elif sign == 'nepravo':
            right_map = {'A': 'B', 'B': 'C', 'C': 'D', 'D': 'A', 'E': 'A'}
            right = right_map.get(node)
            if right in possible_directions:
                target_road = self.get_road(node, right)
                if target_road and current_road not in target_road.forbidden_entry:
                    target_road.forbidden_entry.append(current_road)
                    self.publish_graph_update(
                        target_road.name, 'forbidden_entry', [current_road.name])
                self.get_logger().info(f'Node {node}: right turn to {right} forbidden')

        # ---- НЕ НАЛЕВО ----
        elif sign == 'nelevo':
            left_map = {'A': 'D', 'B': 'A', 'C': 'B', 'D': 'C', 'E': 'D'}
            left = left_map.get(node)
            if left in possible_directions:
                target_road = self.get_road(node, left)
                if target_road and current_road not in target_road.forbidden_entry:
                    target_road.forbidden_entry.append(current_road)
                    self.publish_graph_update(
                        target_road.name, 'forbidden_entry', [current_road.name])
                self.get_logger().info(f'Node {node}: left turn to {left} forbidden')

        # ---- ПАРКОВКА ----
        elif sign == 'parkovka':
            if not current_road.have_parking:
                current_road.have_parking = True
                self.publish_graph_update(current_road.name, 'have_parking', ['yes'])

        # ---- ОСТАНОВКА ----
        elif sign == 'ostanovka':
            if not current_road.have_passengers:
                current_road.have_passengers = True
                self.publish_graph_update(current_road.name, 'have_passengers', ['yes'])

        # ---- ОПАСНОСТЬ / НЕИЗВЕСТНЫЙ ----
        elif sign == 'opasnost':
            self.stop_robot()
        else:
            self.stop_robot()

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