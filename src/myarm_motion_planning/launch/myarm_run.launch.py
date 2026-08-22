# ~/elephant_robots_ws/src/myarm_motion_planning/launch/myarm_run.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='myarm_motion_planning',
            executable='myarm_move_node',
            name='myarm_motion_planner',
            output='screen',
            
            # *** ΔΙΟΡΘΩΜΕΝΟ: Χρήση remappings (πληθυντικός) ***
            remappings=[
                ('robot_description', '/robot_description'),
                ('robot_description_semantic', '/robot_description_semantic'),
                ('robot_description_kinematics', '/robot_description_kinematics'),
                ('robot_description_planning', '/robot_description_planning')
            ],
            
            parameters=[
                {'use_sim_time': True} 
            ]
        ),
    ])
