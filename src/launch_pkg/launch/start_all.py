from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    cam_node = Node(
        package='pkg_img',
        executable='cam_publisher',
        name='camera_node',
        output='screen'
    )

    image_view_node = Node(
        package='pkg_img',
        executable='image_view',
        name='image_viewer_node',
        output='screen'
    )

    antenna_node = Node(
        package='pkg_antenna_modus',
        executable='command_listener',
        name='antenna_command_listener',
        output='screen'
    )

    return LaunchDescription([
        cam_node,
        image_view_node,
        antenna_node
    ])
