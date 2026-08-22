from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. Φόρτωση Configuration
    moveit_config = MoveItConfigsBuilder("myarm_300_pi_thorgripper", package_name="robot_config").to_moveit_configs()

    # 2. Ορισμός Node με "prefix" για νέο παράθυρο
    simple_move_node = Node(
        package="my_robot_control",
        executable="simple_move",
        output="screen",
        # --- Η ΜΑΓΙΚΗ ΓΡΑΜΜΗ ---
        # Αυτό λέει στο ROS: "Μην το τρέξεις εδώ, άνοιξε νέο τερματικό και τρέξ' το εκεί"
        prefix=["gnome-terminal --"], 
        # -----------------------
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            # joint_limits.yaml, δηλαδή robot_description_planning. Χωρίς αυτό
            # ο κόμβος δεν βλέπει τα όρια επιτάχυνσης και η TOTG -- που τρέχει
            # ΜΕΣΑ στο simple_move για τις τροχιές του manifold planner και για
            # όλες τις καρτεσιανές καθόδους -- πέφτει στην προεπιλογή 1 rad/s^2
            # αντί για τα 3.0 του αρχείου. Το move_group τα φορτώνει μόνο του,
            # οπότε το πρόβλημα φαινόταν μόνο στις τροχιές που χρονίζει εδώ.
            moveit_config.joint_limits,
            {'use_sim_time': True}
        ],
    )

    return LaunchDescription([
        simple_move_node
    ])
