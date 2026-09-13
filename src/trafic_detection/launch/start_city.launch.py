from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 1. Нода детектора знаков (YOLO)
        Node(
            package='trafic_detection',
            executable='detector_node',
            name='traffic_detector',
            output='screen',
            parameters=[{
                'confidence': 0.6,
                'roi_x_min_ratio': 0.4,
            }],
            remappings=[
                ('/image_raw', '/camera/image_raw'),
            ],
        ),

        # 2. Оценка дистанции до знака -> /sign_is_close
        Node(
            package='trafic_detection',
            executable='distance_estimator',
            name='distance_estimator',
            output='screen',
        ),

        # 3. Обработчик знаков -> /graph_updates
        Node(
            package='trafic_detection',
            executable='sign_handler_node',
            name='sign_handler_node',
            output='screen',
        ),

        # 4. Следование по линии
        Node(
            package='trafic_detection',
            executable='lf_node',
            name='line_follower',
            output='screen',
            parameters=[{
                'base_linear_speed': 0.15,
                'steering_k': 0.03,
                'search_omega': 0.7,
            }],
        ),
    ])
