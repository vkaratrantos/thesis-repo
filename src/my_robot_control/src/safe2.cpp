#include <memory>
#include <thread>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "vial_pro_system",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);
  std::thread([executor]() { executor->spin(); }).detach();

  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface arm_interface(node, "arm_group");
  MoveGroupInterface gripper_interface(node, "gripper");

  arm_interface.setMaxVelocityScalingFactor(0.2); // Αργή κίνηση για ασφάλεια υγρών
  arm_interface.setGoalPositionTolerance(0.01);

  RCLCPP_INFO(node->get_logger(), "Vial Transfer Pro System Ready.");

  while (rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    std::cout << "\n========== ΕΛΕΓΧΟΣ ΜΕΤΑΦΟΡΑΣ ΦΙΑΛΙΔΙΩΝ ==========" << std::endl;
    std::cout << "1. Move to XYZ (Ευθεία γραμμή & Οριζόντια δαγκάνα)" << std::endl;
    std::cout << "2. Pour Liquid (Περιστροφή Joint 6)" << std::endl;
    std::cout << "3. Gripper (Open/Close)" << std::endl;
    std::cout << "Επιλογή: " << std::flush;

    std::string choice;
    std::getline(std::cin, choice);

    if (choice == "1") {
        // ΖΗΤΗΣΗ ΣΥΝΤΕΤΑΓΜΕΝΩΝ ΓΙΑ ΤΗΝ ΚΙΝΗΣΗ
        std::cout << "Δώσε Στόχο X Y Z (π.χ. 0.2 0.0 0.25): ";
        std::string line;
        std::getline(std::cin, line);
        std::stringstream ss(line);
        double tx, ty, tz;
        if (!(ss >> tx >> ty >> tz)) continue;

        // 1. Καθορισμός της "Τέλειας" Οριζόντιας Στάσης (Orientation Lock)
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = tx;
        target_pose.position.y = ty;
        target_pose.position.z = tz;

        tf2::Quaternion q;
        q.setRPY(0, 1.57, 0); // Οριζόντιος Gripper
        target_pose.orientation = tf2::toMsg(q);

        // 2. Υπολογισμός Καρτεσιανής Διαδρομής (Straight Line)
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        // Υπολογισμός κίνησης ανά 1cm (0.01) χωρίς επιτρεπόμενα "άλματα" (0.0)
        double fraction = arm_interface.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

        if (fraction > 0.95) { // Αν βρέθηκε ευθεία διαδρομή για το >95% της απόστασης
            std::cout << ">>> Ευθεία διαδρομή βρέθηκε. Εκτέλεση..." << std::endl;
            arm_interface.execute(trajectory);
        } else {
            std::cout << ">>> ΑΠΟΤΥΧΙΑ: Δεν είναι δυνατή η ευθεία κίνηση σε αυτό το σημείο." << std::endl;
            std::cout << ">>> Δοκίμασε να αλλάξεις ελαφρώς το ύψος (Z)." << std::endl;
        }
    } 
    else if (choice == "2") {
        // ΠΕΡΙΣΤΡΟΦΗ ΜΟΝΟ ΤΟΥ ΤΕΛΕΥΤΑΙΟΥ ΜΟΤΕΡ ΓΙΑ ΑΔΕΙΑΣΜΑ
        std::cout << ">>> Εκτέλεση περιστροφής (Pouring)..." << std::endl;
        std::vector<double> joints;
        arm_interface.getCurrentState()->copyJointGroupPositions("arm_group", joints);
        
        joints[6] += 1.57; // Στροφή 90 μοίρες
        arm_interface.setJointValueTarget(joints);
        arm_interface.move();
    }
    else if (choice == "3") {
        // ΕΛΕΓΧΟΣ ΔΑΓΚΑΝΑΣ
        std::cout << "1. Open | 2. Close: ";
        std::string g_choice;
        std::getline(std::cin, g_choice);
        std::string target = (g_choice == "1") ? "open" : "closed";
        
        gripper_interface.setNamedTarget(target);
        gripper_interface.move();
    }
  }

  rclcpp::shutdown();
  return 0;
}
