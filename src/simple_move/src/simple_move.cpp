#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h> // Χρειάζεται αυτό το include

void add_floor(moveit::planning_interface::MoveGroupInterface& move_group)
{
    // Δημιουργία του Interface για το περιβάλλον
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // 1. Ορίζουμε το αντικείμενο (Collision Object)
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();
    collision_object.id = "floor"; // Το όνομα του εμποδίου

    // 2. Ορίζουμε το σχήμα (Ένα μεγάλο κουτί)
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 2.0; // 2 μέτρα μήκος
    primitive.dimensions[1] = 2.0; // 2 μέτρα πλάτος
    primitive.dimensions[2] = 0.1; // 10 εκατοστά πάχος

    // 3. Ορίζουμε τη θέση (Pose)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.0;
    box_pose.position.y = 0.0;
    // Το βάζουμε στο z = -0.051 ώστε η επιφάνειά του να είναι λίγο κάτω από το 0
    // (για να μην χτυπάει η βάση του ρομπότ συνεχώς στο πάτωμα και βγάζει error)
    box_pose.position.z = -0.051; 

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    // 4. Το προσθέτουμε στο MoveIt
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    planning_scene_interface.applyCollisionObjects(collision_objects);

    RCLCPP_INFO(rclcpp::get_logger("move_helper"), "Floor added to the scene.");
}

int main(int argc, char *argv[])
{

	rclcpp::init(argc,argv);
	auto const node = std::make_shared<rclcpp::Node>
	(
	"simple_move",
	rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
	);
	
	auto const logger = rclcpp::get_logger("simple_move");
	
	moveit::planning_interface::MoveGroupInterface MoveGroupInterface(node, "arm_group");
	
	tf2::Quaternion tf2_quat;
	tf2_quat.setRPY(0, 0, 0);
	geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(tf2_quat);
	geometry_msgs::msg::Pose GoalPose;
	GoalPose.orientation= msg_quat;
	GoalPose.position.x = 0.1;
	GoalPose.position.y = 0.22;
	GoalPose.position.z = 0.25;	
	
	MoveGroupInterface.setPoseTarget(GoalPose);
	

	//MoveGroupInterface.setNamedTarget("home");

	moveit::planning_interface::MoveGroupInterface::Plan plan1;
	auto const outcome = static_cast<bool>(MoveGroupInterface.plan(plan1));
	
	
	if(outcome)
	{
		MoveGroupInterface.execute(plan1);
	}
	else
	{
		RCLCPP_ERROR(logger, "We were not able to plan and execute!");
	}
	
	rclcpp::shutdown();
	return 0;
}
