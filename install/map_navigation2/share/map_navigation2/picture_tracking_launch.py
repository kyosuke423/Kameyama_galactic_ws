import os
# from rclpy.parameter import Parameter
from launch import LaunchDescription
from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    ld = LaunchDescription()

    namespace = LaunchConfiguration("namespace")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="robot1", description="Top-level namespace"
    )

    map_navigation2_node = Node(
        package='map_navigation2',
        executable='picture_tracking',
        name='picture_tracking',
        namespace=namespace,
        # parameters=[{'robot_name': 'mecanum1'}],
        output='screen',
        emulate_tty=True  #これがないと表示されない。foxy以降はemulate_tty=True
    )
    ld.add_action(declare_namespace_cmd)
    ld.add_action(map_navigation2_node)
    return ld