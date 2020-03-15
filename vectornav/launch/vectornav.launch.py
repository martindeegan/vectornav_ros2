import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        node_name='vectornav_container',
        node_namespace='',
        package='rclcpp_components',
        node_executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='vectornav',
                node_plugin='vn_ros::VectorNavNode',
                node_name='vectornav_node',
                parameters=[{
                        "sensor_port": "/dev/ttyUSB0",
                        "baudrate": 921600,
                        "sample_rate": 200,
                        "topic": "/imu",
                        "frame_id": "imu",
                        "gyroscope_variance": 1e-3,
                        "accelerometer_variance": 1e-3
                }]
            )
        ],
        output='screen',
        emulate_tty=True
    )

    return launch.LaunchDescription([container])
