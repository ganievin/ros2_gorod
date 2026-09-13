#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int64, String
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class TrafficDetector(Node):
    def __init__(self):
        super().__init__('traffic_detector')

        # ==================== ПАРАМЕТРЫ ====================
        self.declare_parameter('model_path', '/home/yana/cvat/plavki/yolov8/runs/detect/yolo_light/weights/best.pt')
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('camera_topic', '/image_raw')
        self.declare_parameter('custom_classes', [0, 1, 2, 3, 4, 5, 6, 7])  
        self.declare_parameter('output_topic', '/vivod')
        self.declare_parameter('string_output_topic', '/detected_sign') 
        self.declare_parameter('area_output_topic', '/sign_area')  # ИЗМЕНЕНО: теперь /sign_area
        self.declare_parameter('image_output_topic', '/detection_image')
        self.declare_parameter('roi_x_min_ratio', 0.4) 

        # Чтение параметров
        model_path = self.get_parameter('model_path').value
        self.confidence = self.get_parameter('confidence').value
        self.custom_classes = self.get_parameter('custom_classes').value
        camera_topic = self.get_parameter('camera_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.string_output_topic = self.get_parameter('string_output_topic').value
        self.area_output_topic = self.get_parameter('area_output_topic').value  # теперь /sign_area
        self.image_output_topic = self.get_parameter('image_output_topic').value
        self.roi_x_min_ratio = self.get_parameter('roi_x_min_ratio').value

        # Словарь классов
        self.class_mapping = {
            0: 'pryamo', 1: 'pravo', 2: 'levo', 3: 'nepravo',
            4: 'parkovka', 5: 'opasnost', 6: 'nelevo', 7: 'ostanovka'
        }

        # Загрузка нейросети YOLO
        self.get_logger().info(f'Загрузка модели: {model_path}')
        self.model = YOLO(model_path)
        self.get_logger().info(f'Модель успешно загружена. Классы модели: {self.model.names}')

        self.bridge = CvBridge()

        # Подписки и Публикации
        self.subscription = self.create_subscription(Image, camera_topic, self.image_callback, 10)
        self.id_publisher = self.create_publisher(Int64, self.output_topic, 10)
        self.string_publisher = self.create_publisher(String, self.string_output_topic, 10)
        self.area_publisher = self.create_publisher(Int64, self.area_output_topic, 10)  # /sign_area
        self.image_publisher = self.create_publisher(Image, self.image_output_topic, 10)

        self.get_logger().info('Нода traffic_detector полностью готова к работе!')
        self.get_logger().info(f'📤 Публикация площади в топик: {self.area_output_topic}')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            h, w = cv_image.shape[:2]

            # Вырезаем правую область кадра
            x_start = int(w * self.roi_x_min_ratio)
            roi_image = cv_image[:, x_start:w]

            # Запуск детекции YOLO
            results = self.model(roi_image, conf=self.confidence, verbose=False)

            # Сохраняем смещение для визуализации
            x_offset = x_start

            # Находим самый большой знак (координаты в ROI)
            largest_id, class_name, area, box_roi = self.find_largest_custom_object_with_box(results)

            if largest_id is not None:
                # Публикуем числовой ID
                id_msg = Int64()
                id_msg.data = largest_id
                self.id_publisher.publish(id_msg)

                # Публикуем текстовое имя
                str_msg = String()
                str_msg.data = class_name  
                self.string_publisher.publish(str_msg)

                # Публикуем площадь (теперь в /sign_area)
                area_msg = Int64()
                area_msg.data = int(area)
                self.area_publisher.publish(area_msg)

                self.get_logger().info(
                    f'Знак обнаружен: {class_name} (ID: {largest_id}) | Площадь: {area:.0f}px',
                    throttle_duration_sec=1.0
                )

            # Визуализация со смещением координат
            annotated = self.draw_annotations_with_offset(cv_image, results, x_offset)
            
            # Рисуем границу ROI
            cv2.line(annotated, (x_start, 0), (x_start, h), (0, 255, 255), 2)
            cv2.putText(annotated, "SIGN ZONE", (x_start + 10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            self.publish_annotated_image(annotated, msg.header)

        except Exception as e:
            self.get_logger().error(f'Ошибка в callback-функции обработки кадра: {e}')

    def find_largest_custom_object_with_box(self, results):
        """ Находит самый крупный объект и возвращает его бокс в ROI координатах """
        largest_area = 0
        largest_id = None
        largest_class_name = "None"
        largest_box = None

        for result in results:
            if not result.boxes:
                continue

            boxes = result.boxes.xyxy.cpu().numpy()
            cls_ids = result.boxes.cls.cpu().numpy().astype(int)

            for box, cls_id in zip(boxes, cls_ids):
                if cls_id not in self.custom_classes:
                    continue

                x1, y1, x2, y2 = box
                area = (x2 - x1) * (y2 - y1)

                if area > largest_area:
                    largest_area = area
                    largest_id = int(cls_id)
                    largest_class_name = self.model.names.get(cls_id, self.class_mapping.get(cls_id, f"class_{cls_id}"))
                    largest_box = box

        return largest_id, largest_class_name, largest_area, largest_box

    def draw_annotations_with_offset(self, image, results, x_offset):
        """ Отрисовка рамок со смещением координат """
        annotated = image.copy()
        for result in results:
            if not result.boxes:
                continue

            boxes = result.boxes.xyxy.cpu().numpy()
            confs = result.boxes.conf.cpu().numpy()
            cls_ids = result.boxes.cls.cpu().numpy().astype(int)

            for box, conf, cls_id in zip(boxes, confs, cls_ids):
                if cls_id not in self.custom_classes:
                    continue

                # Смещаем координаты для отрисовки
                x1, y1, x2, y2 = map(int, box)
                x1 += x_offset
                x2 += x_offset
                
                class_name = self.model.names.get(cls_id, self.class_mapping.get(cls_id, f"class_{cls_id}"))

                # Зеленый бокс знака
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 3)

                # Текст подписи с площадью
                area = (x2 - x1) * (y2 - y1)
                label = f'{class_name} {conf:.2f} | {area:.0f}px'
                cv2.putText(annotated, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        return annotated

    def publish_annotated_image(self, cv_image, header):
        """ Отправка размеченного изображения в ROS-топик """
        try:
            img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            img_msg.header = header
            self.image_publisher.publish(img_msg)
        except Exception as e:
            pass  # отладочная картинка не критична

def main(args=None):
    rclpy.init(args=args)
    node = TrafficDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Нода принудительно остановлена пользователем.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
