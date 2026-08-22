from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    
    # --- ΠΡΟΣΟΧΗ ΕΔΩ ---
    # Αν το πακέτο με τα configs λέγεται "robot_config", το αφήνεις ως έχει.
    # Αν λέγεται π.χ. "mycobot_config", αλλάζεις και τα δύο strings παρακάτω.
    moveit_config = MoveItConfigsBuilder("robot_config", package_name="robot_config").to_moveit_configs()

    # Ορίζουμε τον κόμβο μας
    simple_move_node = Node(
        package="simple_move",          # Το όνομα του πακέτου σου
        executable="simple_move",       # Το όνομα του εκτελέσιμου (από το CMakeLists.txt)
        output="screen",
        parameters=[
            # Εδώ φορτώνουμε τις απαραίτητες παραμέτρου για το MoveIt
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([
        simple_move_node
    ])
