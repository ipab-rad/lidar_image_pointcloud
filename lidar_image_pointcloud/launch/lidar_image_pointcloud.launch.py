import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='lidar_image_pointcloud',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='lidar_image_pointcloud',
                    plugin='sensing::LidarImagePointcloud',
                    name='lidar_image_pointcloud')
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])