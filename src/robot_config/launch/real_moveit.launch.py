import os
import yaml
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# --- Συναρτήσεις φόρτωσης ---
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    moveit_config_pkg = 'robot_config'

    # 1. URDF Load
    urdf_path = "/home/vkaratrantos/elephant_robots_ws/src/myarm_300_pi/urdf/myarm_300_pi_thorgripper.urdf"
    if not os.path.exists(urdf_path):
         raise FileNotFoundError(f"Το αρχείο δεν βρέθηκε: {urdf_path}")
         
    robot_description_config = xacro.process_file(urdf_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # 2. SRDF & Kinematics
    robot_description_semantic_config = load_file(moveit_config_pkg, 'config/myarm_300_pi_thorgripper.srdf')
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}
    
    kinematics_yaml = load_yaml(moveit_config_pkg, 'config/kinematics.yaml')

    # 3. OMPL Planning
    ompl_planning_yaml = load_yaml(moveit_config_pkg, 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config = {'move_group': ompl_planning_yaml}

    # 4. Controllers (ΑΠΑΡΑΙΤΗΤΟ για το Motion Planning)
    # Αντιγραφή από το demo_gazebo.launch.py
    moveit_controllers = {
        'moveit_simple_controller_manager': {
            'controller_names': ['arm_controller', 'gripper_controller'],
            'arm_controller': {
                'action_ns': 'follow_joint_trajectory',
                'type': 'FollowJointTrajectory',
                'default': True,
                'joints': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
            },
            'gripper_controller': {
                'action_ns': 'gripper_cmd',
                'type': 'GripperCommand',
                'default': True,
                'joints': ['endeffector_gripper']
            }
        },
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    # 5. Trajectory Execution (Ρυθμίσεις χρόνου)
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
        # Επειδή δεν έχουμε Gazebo, ενεργοποιούμε την fake εκτέλεση αν είναι δυνατή
        # ή απλά αφήνουμε το Planning να δουλέψει.
    }

    # 6. Rviz Config
    rviz_config_file = os.path.join(get_package_share_directory(moveit_config_pkg), 'config', 'moveit.rviz')

    # --- NODES ---

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    # ΠΡΟΣΟΧΗ: Το joint_state_publisher_gui είναι χρήσιμο για έλεγχο,
    # αλλά όταν κάνεις Motion Planning, το MoveIt θέλει να ελέγχει αυτό τις αρθρώσεις.
    # Αν θες να κουνάς το ρομπότ με το MoveIt, πρέπει να το έχεις ανοιχτό αλλά να μην
    # κάνει conflict. Για αρχή το αφήνουμε.
    #jsp_gui_node = Node(
    #    package='joint_state_publisher_gui',
    #    executable='joint_state_publisher_gui',
    #    name='joint_state_publisher_gui'
    #)

    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers, # <--- Η ΒΑΣΙΚΗ ΠΡΟΣΘΗΚΗ
            {'use_sim_time': False},
            {'publish_robot_description_semantic': True}
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
            {'use_sim_time': False}
        ],
        arguments=['-d', rviz_config_file]
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base_link']
    )

# ... (Όλοι οι κόμβοι είναι ορισμένοι πάνω) ...
    # ΑΥΤΟ ΕΙΝΑΙ ΤΟ BLOCK ΠΟΥ ΠΡΕΠΕΙ ΝΑ ΤΡΕΞΕΙ

    return LaunchDescription([
        static_tf,       # Δημοσιεύει το frame world -> base_link
        rsp_node,        # Μετατρέπει τις γωνίες του Pi σε 3D μοντέλο
        run_move_group_node, # Ο σχεδιαστής
        rviz_node        # Η οθόνη
    ])

