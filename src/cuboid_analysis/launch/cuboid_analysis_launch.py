import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # C++ノードの起動
        Node(
            package='cuboid_analysis',
            executable='ave_cov',
            name='ave_cov',
            output='screen'
        ),
        
        # Pythonスクリプトの起動
        Node(
            package='cuboid_analysis',
            executable='d_op.py',
            name='d_op',
            output='screen',
        )
    ])
