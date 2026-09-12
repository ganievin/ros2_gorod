#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, Bool

class DistanceEstimator(Node):
    def __init__(self):
        super().__init__('distance_estimator')

        # Параметры
        self.declare_parameter('area_threshold', 15000)
        self.declare_parameter('id_topic', '/vivod')
        self.declare_parameter('area_topic', '/sign_area')
        self.declare_parameter('output_topic', '/sign_is_close')

        self.area_threshold = self.get_parameter('area_threshold').value
        self.id_topic = self.get_parameter('id_topic').value
        self.area_topic = self.get_parameter('area_topic').value
        self.output_topic = self.get_parameter('output_topic').value

        # Храним последний ID и площадь
        self.last_id = None
        self.last_area = None

        # Подписки
        self.sub_id = self.create_subscription(Int64, self.id_topic, self.on_id_received, 10)
        self.sub_area = self.create_subscription(Int64, self.area_topic, self.on_area_received, 10)

        # Публикация статуса
        self.publisher = self.create_publisher(Bool, self.output_topic, 10)

        self.get_logger().info(f'✅ distance_estimator запущен! Порог: {self.area_threshold}')

    def on_area_received(self, msg):
        self.last_area = msg.data
        self.check_and_publish()

    def on_id_received(self, msg):
        self.last_id = msg.data
        self.check_and_publish()

    def check_and_publish(self):
        if self.last_id is not None and self.last_area is not None:
            area = self.last_area
            is_close = area > self.area_threshold
            status = "БЛИЗКО ✅" if is_close else "ДАЛЕКО ❌"

            bool_msg = Bool()
            bool_msg.data = is_close
            self.publisher.publish(bool_msg)

            self.get_logger().info(f'📊 ID: {self.last_id} | Площадь: {area}px | {status}')

            self.last_id = None
            self.last_area = None

def main(args=None):
    rclpy.init(args=args)
    node = DistanceEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Нода остановлена пользователем.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

