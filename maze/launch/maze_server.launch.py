from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    maze_node = Node(
        package='maze',
        executable='maze_action_server_node',
        output='screen',
        remappings=[
            ('/scan', '/dolly/laser_scan'),
            ('/odom', '/dolly/odom'),
            ('/cmd_vel', '/dolly/cmd_vel')
        ],
        parameters=[
            {"laserMsgRanges": 100},  ## declare_parameter로 설정한 파라미터명 : 바꿀 파라미터 // for dolly
            {"turnLimit": 0.067},
            {"distanceLimit": 0.8},
            {"speedLimit": 0.3}
        ]
    )

    return LaunchDescription([
        maze_node,
    ])
