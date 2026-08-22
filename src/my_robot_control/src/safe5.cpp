#include <memory>
#include <thread>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <iomanip>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2/LinearMath/Quaternion.h>

// Συνάρτηση για την εκτύπωση της τρέχουσας θέσης
void printPose(const geometry_msgs::msg::Pose& pose) {
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "[ΤΡΕΧΟΥΣΑ ΘΕΣΗ]: X=" << pose.position.x 
              << ", Y=" << pose.position.y 
              << ", Z=" << pose.position.z << std::endl;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("cartesian_move_node");

  // Multi-threaded executor για να μην κολλάει το node κατά την εκτέλεση
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);
  std::thread([executor]() { executor->spin(); }).detach();

  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface arm_interface(node, "arm_group"); 
  MoveGroupInterface gripper_interface(node, "gripper"); 

  // Ρυθμίσεις ταχύτητας
  arm_interface.setMaxVelocityScalingFactor(0.3); 
  arm_interface.setMaxAccelerationScalingFactor(0.3);
  gripper_interface.setMaxVelocityScalingFactor(0.5);

  // --- ΠΡΟΣΘΗΚΗ ΠΑΤΩΜΑΤΟΣ ΣΤΟ PLANNING SCENE ---
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = arm_interface.getPlanningFrame();
  collision_object.id = "floor";
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions = {2.0, 2.0, 0.02};
  geometry_msgs::msg::Pose box_pose;
  box_pose.position.z = -0.05; 
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;
  planning_scene_interface.applyCollisionObjects({collision_object});

  // Ορισμός οριζόντιου προσανατολισμού (Pitch 90 μοίρες)
  tf2::Quaternion q_horizontal;
  q_horizontal.setRPY(0.0, 1.5708, 0.0); 

  // --- HELPER FUNCTION ΓΙΑ ΚΑΡΤΕΣΙΑΝΕΣ ΚΙΝΗΣΕΙΣ ---
  // Διατηρεί σταθερό τον οριζόντιο προσανατολισμό κατά την κίνηση
  auto move_cartesian = [&](double x, double y, double z) -> bool {
      std::vector<geometry_msgs::msg::Pose> waypoints;
      geometry_msgs::msg::Pose target_pose;
      target_pose.position.x = x;
      target_pose.position.y = y;
      target_pose.position.z = z;
      target_pose.orientation.x = q_horizontal.x();
      target_pose.orientation.y = q_horizontal.y();
      target_pose.orientation.z = q_horizontal.z();
      target_pose.orientation.w = q_horizontal.w();
      
      waypoints.push_back(target_pose);
      moveit_msgs::msg::RobotTrajectory trajectory;
      
      // Υπολογισμός ευθείας διαδρομής
      double fraction = arm_interface.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
      
      if (fraction >= 0.95) { 
          moveit::planning_interface::MoveGroupInterface::Plan plan;
          plan.trajectory_ = trajectory; 
          arm_interface.execute(plan);
          rclcpp::sleep_for(std::chrono::milliseconds(500)); 
          return true;
      }
      return false;
  };

  RCLCPP_INFO(node->get_logger(), "Εκκίνηση διαδικασίας...");

  // --- 1. ΑΡΧΙΚΟΠΟΙΗΣΗ (Home Position) ---
  std::cout << ">>> Μετάβαση σε αρχική οριζόντια θέση..." << std::endl;
  geometry_msgs::msg::Pose start_pose;
  start_pose.position.x = 0.0; start_pose.position.y = 0.2; start_pose.position.z = 0.2; 
  start_pose.orientation.x = q_horizontal.x(); start_pose.orientation.y = q_horizontal.y();
  start_pose.orientation.z = q_horizontal.z(); start_pose.orientation.w = q_horizontal.w();
  arm_interface.setPoseTarget(start_pose);
  arm_interface.move();

  // --- 2. ΚΥΡΙΟΣ ΒΡΟΧΟΣ ΕΝΤΟΛΩΝ ---
  while (rclcpp::ok()) {
    std::cout << "\n================================================" << std::endl;
    arm_interface.setStartStateToCurrentState();
    printPose(arm_interface.getCurrentPose().pose);

    std::cout << "Επιλογές:\n"
              << " - 'O' : Άνοιγμα δαγκάνας\n"
              << " - 'C' : Κλείσιμο δαγκάνας\n"
              << " - 'P X Y Z' : Pick φιαλιδίου (π.χ. P 0.25 0.0 0.1)\n"
              << " - X Y Z     : Απλή κίνηση σε ευθεία\n"
              << "Εντολή: ";
              
    std::string line;
    std::getline(std::cin, line);
    if (line.empty()) continue;

    // Manual Gripper Control
    if (line == "O" || line == "o") {
        gripper_interface.setNamedTarget("open"); // Βεβαιώσου για το όνομα (Open/open)
        gripper_interface.move();
        continue;
    } 
    if (line == "C" || line == "c") {
        gripper_interface.setNamedTarget("closed"); 
        gripper_interface.move();
        continue;
    }

    // --- ΔΙΑΔΙΚΑΣΙΑ PICK X Y Z ---
    if (line[0] == 'P' || line[0] == 'p') {
        std::stringstream ss(line.substr(1));
        double px, py, pz;
        if (ss >> px >> py >> pz) {
            std::cout << "\n>>> ΕΚΤΕΛΕΣΗ PICK ΣΤΟ (" << px << ", " << py << ", " << pz << ")" << std::endl;
            
            // 1. Προσέγγιση (10cm πάνω από το φιαλίδιο)
            std::cout << " -> Προσέγγιση από πάνω..." << std::endl;
            if(!move_cartesian(px, py, pz + 0.10)) { std::cout << "Αποτυχία προσέγγισης!\n"; continue; }

            // 2. Άνοιγμα δαγκάνας
            std::cout << " -> Άνοιγμα δαγκάνας..." << std::endl;
            gripper_interface.setNamedTarget("open");
            gripper_interface.move();

            // 3. Κάθοδος (Ευθεία βουτιά)
            std::cout << " -> Κάθοδος στο φιαλίδιο..." << std::endl;
            if(!move_cartesian(px, py, pz)) { std::cout << "Αποτυχία καθόδου!\n"; continue; }

            // 4. Κλείσιμο δαγκάνας (Πιάσιμο)
            std::cout << " -> Κλείσιμο δαγκάνας..." << std::endl;
            gripper_interface.setNamedTarget("closed");
            gripper_interface.move();

            // 5. Ανάταση (Ευθεία άνοδος)
            std::cout << " -> Ανάταση με το φιαλίδιο..." << std::endl;
            move_cartesian(px, py, pz + 0.10);

            std::cout << ">>> PICK ΟΛΟΚΛΗΡΩΘΗΚΕ!" << std::endl;
        }
        continue;
    }

    // Απλή κίνηση X Y Z
    std::stringstream ss(line);
    double tx, ty, tz;
    if (ss >> tx >> ty >> tz) {
        move_cartesian(tx, ty, tz);
    }
  }

  rclcpp::shutdown();
  return 0;
}
