import os
import yaml
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

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

    # =========================================================================
    # 1. Load Robot URDF
    # =========================================================================
    urdf_path = "/home/vkaratrantos/elephant_robots_ws/src/myarm_300_pi/urdf/myarm_300_pi_thorgripper.urdf"
    if not os.path.exists(urdf_path):
         urdf_path = os.path.join(get_package_share_directory("mycobot_description"), "urdf/myarm_300_pi/myarm_300_pi.urdf")
    robot_description_config = xacro.process_file(urdf_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # =========================================================================
    # 2. Load Core MoveIt Configs (SRDF, Kinematics, Limits)
    # =========================================================================
    robot_description_semantic_config = load_file(moveit_config_pkg, 'config/myarm_300_pi_thorgripper.srdf')
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}
    
    kinematics_yaml = load_yaml(moveit_config_pkg, 'config/kinematics.yaml')
    
    # Φορτώνουμε τα joint limits που φτιάξαμε
    joint_limits_yaml = load_yaml(moveit_config_pkg, 'config/joint_limits.yaml')
    
    # [ΝΕΟ] Ενώνουμε τα Joint Limits με τα Καρτεσιανά (Cartesian) Limits για τον Pilz
    robot_description_planning_config = {
        'robot_description_planning': {
            **joint_limits_yaml,  # Βάζει μέσα τα joint limits
            'cartesian_limits': { # Ορίζουμε τα όρια για τη γραμμική κίνηση στο χώρο
                'max_trans_vel': 0.5,    # m/s (Μέγιστη ταχύτητα μετάφρασης)
                'max_trans_acc': 1.0,    # m/s^2 (Επιτάχυνση)
                'max_trans_dec': -1.0,   # m/s^2 (Επιβράδυνση)
                'max_rot_vel': 1.5,      # rad/s (Μέγιστη ταχύτητα περιστροφής)
                'max_rot_acc': 2.0,
                'max_rot_dec': -2.0,
            }
        }
    }

    # =========================================================================
    # 3. Load Planning Pipelines (OMPL, Pilz)
    # =========================================================================
    ompl_planning_yaml = load_yaml(moveit_config_pkg, 'config/ompl_planning.yaml')
    
    # The 'ompl_stomp' pipeline is disabled: stomp_moveit is not installed on
    # this machine (not in /opt/ros/humble, not in ws_moveit2, no apt package),
    # so move_group threw "StompPlanner ... does not exist" on every startup and
    # then "Failed to initialize planning pipeline 'ompl_stomp'". The configs
    # ompl_stomp_planning.yaml / stomp_planning.yaml are still in config/ --
    # re-add the two loads and the pipeline entries below if stomp is installed.

    # [ΝΕΟ] Ορίζουμε τον Pilz απευθείας στο Python (διορθωμένο για ROS 2)
    pilz_config = {
        'planning_plugin': 'pilz_industrial_motion_planner/CommandPlanner',
        'request_adapters': 'default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision',
        'default_planner_config': 'LIN',
        'arm_group': {
            'planner_configs': ['PTP', 'LIN', 'CIRC']
        }
    }

    pipeline_config = {
        # [ΑΛΛΑΓΗ] Προσθέσαμε το 'ompl_stomp' στη λίστα των ενεργών pipelines
        'planning_pipelines': ['ompl', 'pilz_industrial_motion_planner'],
        'default_planning_pipeline': 'ompl',
        'ompl': ompl_planning_yaml,
        'pilz_industrial_motion_planner': pilz_config,
    }

    # =========================================================================
    # 4. Controllers Configuration
    # =========================================================================
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
                'action_ns': 'follow_joint_trajectory',
                'type': 'FollowJointTrajectory',
                'default': True,
                'joints': ['endeffector_gripper']
            }
        },
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    rviz_config_file = os.path.join(get_package_share_directory(moveit_config_pkg), 'config', 'moveit.rviz')

    # =========================================================================
    # 5. Nodes
    # =========================================================================
    
    # Joint State Publisher
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        # 'rate' defaults to 10 Hz, which decimates everything published on
        # fake_joint_states down to 10 poses per second before RViz ever sees
        # it. A 3 s move then arrives as ~30 poses with up to 25 deg between
        # them, which looks like dropped frames but is simply nothing new to
        # draw. Measured on /joint_states: exactly 10.0 Hz for every move.
        parameters=[{'source_list': ['fake_joint_states'], 'rate': 100}]
    )

    # Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )
    
    # Anchor TF
    anchor_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='anchor_tf',
        arguments=['0.0', '-0.06', '0.0', '0.0', '0.0', '0.0', 'world', 'marker_base']
    )
    
    # Move Group Node
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            robot_description_planning_config,  # <-- ΑΛΛΑΓΗ ΕΔΩ: Περνάμε το νέο config
            pipeline_config,         
            trajectory_execution,
            moveit_controllers, 
            {'use_sim_time': False},
            # Ο ρυθμός με τον οποίο δημοσιεύεται το /monitored_planning_scene.
            #
            # Η προεπιλογή είναι 4 Hz. Το RViz σχεδιάζει από εκεί ό,τι ανήκει
            # στη σκηνή -- και κυρίως τα ATTACHED objects, δηλαδή τον σωλήνα
            # όταν τον κρατάει η αρπάγη. Στα 4 Hz ο σωλήνας ενημερώνεται δύο με
            # τέσσερις φορές το δευτερόλεπτο ενώ ο βραχίονας κινείται στα 100
            # Hz μέσω TF, οπότε φαίνεται να μένει πίσω ή να αναβοσβήνει.
            #
            # Μετρημένο στο βίντεο: 2.9 Hz οπτικές ενημερώσεις, με την εικόνα
            # παγωμένη κατά μέσο όρο 504 ms κάθε φορά.
            #
            # Στα 30 Hz ταιριάζει με το Frame Rate του RViz. Η σκηνή εδώ έχει
            # μόνο επτά απλά σχήματα, οπότε το κόστος είναι αμελητέο.
            #
            # ΠΡΟΣΟΧΗ: το hz από μόνο του δεν αρκεί -- είναι ΑΝΩΤΑΤΟ όριο, και η
            # σκηνή δημοσιεύεται μόνο όταν ΑΛΛΑΖΕΙ. Χωρίς το publish_state_updates
            # οι αλλαγές της κατάστασης του ρομπότ δεν προκαλούν δημοσίευση, οπότε
            # το μόνο που την ενεργοποιούσε ήταν ο updater των collision objects
            # του simple_move ανά 500 ms. Μετρημένο: ακριβώς 2.0 Hz.
            {
                'publish_planning_scene': True,
                'publish_geometry_updates': True,
                'publish_state_updates': True,
                'publish_transforms_updates': True,
                'publish_planning_scene_hz': 30.0,
            },
        ],
    )

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        parameters=[
            robot_description,
            robot_description_semantic,
            pipeline_config,
            kinematics_yaml,
            {'use_sim_time': False}
        ],
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        jsp_node,
        rsp_node,
        anchor_tf,
        run_move_group_node,
        rviz_node
    ])
