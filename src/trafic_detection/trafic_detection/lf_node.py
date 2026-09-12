#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from std_msgs.msg import Header
import cv_bridge

_bridge = cv_bridge.CvBridge()

class LineFollower(Node):
    """
    Нода: читает изображение, выделяет линию в нижней части кадра,
    считает поперечную ошибку и публикует Twist в /cmd_vel.
    Управление движением: сервисы /start_follower и /stop_follower.
    """

    def __init__(self) -> None:
        super().__init__('line_follower_core')

        # --- параметры (можно задавать через ros2 param/params-file) ---
        # источники/выходы
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('use_compressed', False)      # True если подписка на CompressedImage
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # ROI (доли кадра 0..1)
        self.declare_parameter('roi_y_min', 0.65)
        self.declare_parameter('roi_y_max', 1.00)
        self.declare_parameter('roi_x_min', 0.00)
        self.declare_parameter('roi_x_max', 1.00)

        # сегментация: 'hsv' или 'bgr' + пороги
        self.declare_parameter('mode', 'bgr')
        self.declare_parameter('lower', [0, 0, 0])
        self.declare_parameter('upper', [70, 70, 70])

        # морфология/площадь
        self.declare_parameter('min_area', 1000)
        self.declare_parameter('morph_kernel', 3)

        # управление
        self.declare_parameter('steering_k', 0.03)          # рад/с на пиксель ошибки
        self.declare_parameter('base_linear_speed', 0.18)   # м/с
        self.declare_parameter('timer_period', 0.05)        # сек (20 Гц)
        self.declare_parameter('max_omega', 2.0)            # ограничитель |angular.z|
        self.declare_parameter('lost_behavior', 'stop')     # 'stop' или 'spin'
        self.declare_parameter('search_omega', 0.6)         # рад/с при 'spin'

        # отладочная картинка для RViz
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('debug_topic', '/line_follower/debug_image/compressed')

        # чтение параметров
        p = self.get_parameter
        self.image_topic     = p('image_topic').value
        self.use_compressed  = bool(p('use_compressed').value)
        self.cmd_vel_topic   = p('cmd_vel_topic').value

        self.roi_y_min = float(p('roi_y_min').value)
        self.roi_y_max = float(p('roi_y_max').value)
        self.roi_x_min = float(p('roi_x_min').value)
        self.roi_x_max = float(p('roi_x_max').value)

        self.mode  = p('mode').value
        self.lower = np.array(p('lower').value, dtype=np.uint8)
        self.upper = np.array(p('upper').value, dtype=np.uint8)

        self.min_area = int(p('min_area').value)
        self.morph_kernel = int(p('morph_kernel').value)

        self.k = float(p('steering_k').value)
        self.v = float(p('base_linear_speed').value)
        self.period = float(p('timer_period').value)
        self.max_omega = float(p('max_omega').value)
        self.lost_behavior = p('lost_behavior').value
        self.search_omega  = float(p('search_omega').value)

        # паблишеры/сабскрайберы
        self.pub_twist = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        dbg_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.pub_dbg = self.create_publisher(CompressedImage, p('debug_topic').value, dbg_qos) \
                        if bool(p('publish_debug').value) else None

        if self.use_compressed:
            self.sub = self.create_subscription(CompressedImage, self.image_topic,
                                                self.on_image_compressed, qos_profile_sensor_data)
            self.get_logger().info(f"Subscribing (compressed): {self.image_topic}")
        else:
            self.sub = self.create_subscription(Image, self.image_topic,
                                                self.on_image_raw, qos_profile_sensor_data)
            self.get_logger().info(f"Subscribing (raw): {self.image_topic}")

        # сервисы старт/стоп
        self.create_service(Empty, 'start_follower', self.on_start)
        self.create_service(Empty, 'stop_follower',  self.on_stop)

        # таймер
        self.timer = self.create_timer(self.period, self.on_timer)

        # состояние
        self._last_img = None
        self._should_move = False

        self.get_logger().info(
            f"line_follower_core: v={self.v:.2f} m/s, k={self.k:.4f}, ROI y=[{self.roi_y_min:.2f},{self.roi_y_max:.2f}] x=[{self.roi_x_min:.2f},{self.roi_x_max:.2f}]"
        )

    # ---- сервисы ----
    def on_start(self, req, res):
        self._should_move = True
        return res

    def on_stop(self, req, res):
        self._should_move = False
        self.pub_twist.publish(Twist())
        return res

    # ---- входные изображения ----
    def on_image_compressed(self, msg: CompressedImage):
        buff = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buff, cv2.IMREAD_COLOR)
        if frame is not None:
            self._last_img = frame

    def on_image_raw(self, msg: Image):
        self._last_img = _bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    # ---- основной цикл ----
    def on_timer(self):
        if self._last_img is None:
            return

        img = self._last_img
        h, w = img.shape[:2]

        y0 = int(np.clip(self.roi_y_min, 0, 1) * h)
        y1 = int(np.clip(self.roi_y_max, 0, 1) * h)
        x0 = int(np.clip(self.roi_x_min, 0, 1) * w)
        x1 = int(np.clip(self.roi_x_max, 0, 1) * w)

        roi = img[y0:y1, x0:x1]

        # сегментация
        if self.mode == 'hsv':
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.lower, self.upper)
        else:
            mask = cv2.inRange(roi, self.lower, self.upper)

        # морфология
        ksz = max(1, self.morph_kernel)
        k = np.ones((ksz, ksz), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k, iterations=1)

        M = cv2.moments(mask, binaryImage=True)
        has_line = M['m00'] > self.min_area

        twist = Twist()
        overlay = img.copy()

        if has_line:
            cx = int(M['m10'] / M['m00']) + x0
            err = cx - (w // 2)         # пиксели
            omega = float(-self.k * err)
            omega = max(-self.max_omega, min(self.max_omega, omega))

            if self._should_move:
                twist.linear.x = self.v
                twist.angular.z = omega

            # отрисовка
            if self.pub_dbg:
                cv2.circle(overlay, (cx, (y0 + y1)//2), 6, (0,0,255), -1)
                cv2.line(overlay, (w//2, y0), (w//2, y1), (0,255,0), 2)
        else:
            # потеряли линию
            if self._should_move:
                if self.lost_behavior == 'spin':
                    twist.linear.x = 0.0
                    twist.angular.z = self.search_omega
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0

        self.pub_twist.publish(twist)

        if self.pub_dbg:
            cv2.rectangle(overlay, (x0, y0), (x1, y1), (255,0,0), 2)
            ok, buff = cv2.imencode('.jpg', overlay, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if ok:
                c = CompressedImage()
                c.header = Header()
                c.format = 'jpeg'
                c.data = buff.tobytes()
                self.pub_dbg.publish(c)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
