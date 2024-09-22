import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    """Generate launch description with multiple components, including use_sim_time and camera_names parameters."""
    
    # Declare the use_sim_time argument
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use rosbag clock if true'
    )

    # Declare the camera_names argument (string vector)
    camera_names_arg = DeclareLaunchArgument(
        'camera_names',
        default_value="['fsp_l', 'lspf_r', 'lspr_l', 'rsp_l', 'rspf_l', 'rspr_r']",
        description='List of camera names (string vector)'
    )

    # Define the ComposableNodeContainer with the use_sim_time and camera_names parameters
    container = ComposableNodeContainer(
        name='lidar_image_pointcloud',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='lidar_image_pointcloud',
                plugin='sensing::LidarImagePointcloud',
                name='lidar_image_pointcloud',
                parameters=[
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                    {'camera_names': LaunchConfiguration('camera_names')}  # Pass the string vector parameter
                ]
            )
        ],
        output='screen',
    )

    return LaunchDescription([
        use_sim_time_arg,  # Add the use_sim_time argument to the launch description
        camera_names_arg,  # Add the camera_names argument
        container
    ])
