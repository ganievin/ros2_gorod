from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Нода детектора знаков (YOLO)
        Node(
            package='trafic_detection',
            executable='detector',
            name='traffic_detector',
            output='screen',
            parameters=[{
                'confidence': 0.6,
                'roi_x_min_ratio': 0.4
            }]
        ),
        # 2. Нода следования по линии
        Node(
            package='trafic_detection',
            executable='lf_node',
            name='line_follower',
            output='screen',
            parameters=[{
                'base_linear_speed': 0.15, # Начальная безопасная скорость
                'steering_k': 0.03,        # Коэффициент руления
                'search_omega': 0.7        # Скорость поворота на перекрестке
            }]
        )
    ])
