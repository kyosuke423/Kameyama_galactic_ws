from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cuboid_analysis',
            executable='d_op.py',
            name='d_op',
            output='screen',
            #parameters=[{'param_name': 'param_value'}],  # 必要なパラメータがあればここに追加
            # `exec` を使って Python スクリプトを実行するように指定します
            executable='/usr/bin/python3',
            arguments=['/home/dars/kameyama_galactic_ws/src/cuboid_analysis/scripts/d_op.py']
        ),
        Node(
            package='cuboid_analysis',
            executable='ave_cov',
            name='ave_cov',
            output='screen',
        ),
    ])

