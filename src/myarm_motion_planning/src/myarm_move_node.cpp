#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <thread>
#include <memory>

void move_to_pose(std::shared_ptr<rclcpp::Node> node)
{
  // 1. Ρύθμιση MoveGroupInterface
  // Χρησιμοποιούμε το PLANNING_GROUP "arm_group" όπως επιβεβαιώθηκε
  static const std::string PLANNING_GROUP = "arm_group"; 
  
  // Το MoveGroupInterface χρειάζεται έναν κόμβο για να ξεκινήσει
  moveit::planning_interface::MoveGroupInterface move_group_interface(node, PLANNING_GROUP);

  RCLCPP_INFO(node->get_logger(), "MoveGroupInterface initialized for group: %s", PLANNING_GROUP.c_str());
  
  // 2. Ορισμός Θέσης Στόχου (Target Pose)
  geometry_msgs::msg::Pose target_pose;
  
  // Ορισμός Προσανατολισμού (Orientation - Quaternion)
  // Ελέγξτε αν ο προσανατολισμός είναι έγκυρος για το ρομπότ σας
  target_pose.orientation.w = 1.0; 
  target_pose.orientation.x = 0.0;
  target_pose.orientation.y = 0.0;
  target_pose.orientation.z = 0.0;
  
  // Ορισμός Θέσης (Position - Meters)
  // Αυτές οι συντεταγμένες (X, Y, Z) πρέπει να είναι εντός του Workspace
  target_pose.position.x = 0.3; 
  target_pose.position.y = 0.0;
  target_pose.position.z = 0.5;

  move_group_interface.setPoseTarget(target_pose);

  RCLCPP_INFO(node->get_logger(), "Attempting to plan to target pose...");
  
  // 3. Σχεδιασμός (Planning)
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  // Χρησιμοποιούμε ένα timeout για να μην κολλήσει
  move_group_interface.setPlanningTime(5.0); 

  // Προσπάθεια εύρεσης πλάνου
  bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  // 4. Εκτέλεση (Execution)
  if (success) {
    RCLCPP_INFO(node->get_logger(), "Planning successful! Executing...");
    move_group_interface.execute(my_plan);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Planning failed! Check if the target is reachable.");
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  // *** Η ΚΡΙΣΙΜΗ ΔΙΟΡΘΩΣΗ ΓΙΑ ΤΗ ΦΟΡΤΩΣΗ ΠΑΡΑΜΕΤΡΩΝ ***
  // Αυτό είναι απαραίτητο για να βρει ο κόμβος τις παραμέτρους (robot_description, κλπ)
  // που φορτώθηκαν στο root namespace από το launch file του MoveIt/Gazebo.
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.allow_undeclared_parameters(true);
  
  // Δημιουργία του κόμβου: το δεύτερο όρισμα ("") διασφαλίζει ότι δεν χρησιμοποιείται 
  // το όνομα του κόμβου ως namespace, ψάχνοντας έτσι στο root (/).
  std::shared_ptr<rclcpp::Node> node = 
      rclcpp::Node::make_shared("myarm_motion_planner", "", node_options);
  
  // Χρησιμοποιούμε έναν SingleThreadedExecutor για να εκτελέσουμε callbacks 
  // του MoveGroupInterface
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  
  // Εκτέλεση του executor σε ξεχωριστό thread. Αυτό επιτρέπει στον κύριο κώδικα 
  // να προχωρήσει στον σχεδιασμό, ενώ ο κόμβος παραμένει ζωντανός.
  std::thread([&executor]() { executor.spin(); }).detach(); 
  
  // Καλούμε τη συνάρτηση κίνησης
  move_to_pose(node); 
  
  rclcpp::shutdown();
  return 0;
}
