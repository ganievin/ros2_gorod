#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
import serial
import sys
import re

class STMComm(Node):
    def __init__(self):
        super().__init__('STM_comm_node')
        
        # Параметры (можно переопределять через launch-файл)
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        
        # Публикатор одометрии
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # Открываем последовательный порт
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1.0)
            self.get_logger().info(f"Serial port {port} opened successfully")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {port}: {e}")
            sys.exit(1)
        
        # Таймер для чтения данных (10 Гц)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Буфер для накопления строки
        self.buffer = ""
        
        self.get_logger().info("Odom Publisher node started")
    
    def timer_callback(self):
        """Читает данные из сериального порта и публикует одометрию"""
        try:
            # Читаем все доступные байты
            data = self.ser.read(self.ser.in_waiting or 1)
            if data:
                # Декодируем и добавляем в буфер
                self.buffer += data.decode('utf-8', errors='ignore')
                
                # Разбиваем по символу новой строки
                lines = self.buffer.split('\n')
                # Последний элемент - неполная строка, оставляем в буфере
                self.buffer = lines[-1]
                
                # Обрабатываем полные строки
                for line in lines[:-1]:
                    self.process_line(line.strip())
                    
        except serial.SerialException as e:
            self.get_logger().error(f"Serial read error: {e}")
        except UnicodeDecodeError as e:
            self.get_logger().warn(f"Unicode decode error: {e}")
    
    def process_line(self, line):
        """Парсит строку вида 'x,y,theta' и публикует Odometry"""
        if not line:
            return
        
        # Ожидаемый формат: "x,y,theta"
        pattern = r'^-?\d+\.?\d*,-?\d+\.?\d*,-?\d+\.?\d*$'
        if not re.match(pattern, line):
            self.get_logger().warn(f"Invalid format: {line}")
            return
        
        try:
            parts = line.split(',')
            if len(parts) != 3:
                return
            
            x = float(parts[0])
            y = float(parts[1])
            theta = float(parts[2])
            
            # Создаём сообщение Odometry
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = self.frame_id
            odom_msg.child_frame_id = self.child_frame_id
            
            # Позиция
            odom_msg.pose.pose.position.x = x
            odom_msg.pose.pose.position.y = y
            odom_msg.pose.pose.position.z = 0.0
            
            # Ориентация (из theta)
            q = self.euler_to_quaternion(0.0, 0.0, theta)
            odom_msg.pose.pose.orientation = q
            
            # Ковариацию пока оставляем нулевой (можно будет добавить позже)
            # odom_msg.pose.covariance = [0.0] * 36
            # odom_msg.twist.covariance = [0.0] * 36
            
            # Публикуем
            self.odom_pub.publish(odom_msg)
            
            # Логируем для отладки (можно закомментировать потом)
            self.get_logger().debug(f"Published odom: x={x:.3f}, y={y:.3f}, theta={theta:.3f}")
            
        except ValueError as e:
            self.get_logger().warn(f"Parse error: {e} on line: {line}")
    
    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """Преобразует углы Эйлера в кватернион"""
        from geometry_msgs.msg import Quaternion
        import math
        q = Quaternion()
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q
    
    def destroy_node(self):
        """Закрываем порт при завершении"""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial port closed")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = STMComm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
