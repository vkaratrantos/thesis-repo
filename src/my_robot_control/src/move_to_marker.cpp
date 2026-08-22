#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("marker_follower");
  
  // 1. Ρύθμιση TF Listener (Για να βρίσκουμε τις συντεταγμένες)
  auto tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // 2. Ρύθμιση MoveIt
  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface arm_interface(node, "arm_group");
  
  // Μέγιστη ταχύτητα
  arm_interface.setMaxVelocityScalingFactor(1.0);
  arm_interface.setMaxAccelerationScalingFactor(1.0);

  // Thread για να τρέχει το ROS στο παρασκήνιο
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  std::cout << "--- ΣΥΣΤΗΜΑ ΕΤΟΙΜΟ ---" << std::endl;
  std::cout << "Βεβαιώσου ότι η κάμερα βλέπει το Marker 0 (Άγκυρα)!" << std::endl;

  while (rclcpp::ok()) {
      std::cout << "\nΠοιο Marker ID να πιάσω; (π.χ. 1): ";
      int target_id;
      std::cin >> target_id;
      
      std::string target_frame = "marker_" + std::to_string(target_id);

      try {
          // Ζητάμε: "Πού είναι το Marker X σε σχέση με το World;"
          // Το ROS θα υπολογίσει: World -> Anchor -> Camera -> Target
          geometry_msgs::msg::TransformStamped t;
          t = tf_buffer->lookupTransform("world", target_frame, tf2::TimePointZero, tf2::durationFromSec(1.0));

          std::cout << "Στόχος βρέθηκε στο: X=" << t.transform.translation.x 
                    << " Y=" << t.transform.translation.y << std::endl;

          // 3. ΕΤΟΙΜΑΣΙΑ ΚΙΝΗΣΗΣ
          geometry_msgs::msg::Pose target_pose;
          target_pose.position.x = t.transform.translation.x;
          target_pose.position.y = t.transform.translation.y;
          // ΠΑΜΕ 10 ΕΚΑΤΟΣΤΑ ΠΑΝΩ ΑΠΟ ΤΟΝ ΣΤΟΧΟ (Για ασφάλεια)
          target_pose.position.z = t.transform.translation.z + 0.10;

          // Σταθερός προσανατολισμός (Gripper προς τα κάτω)
          tf2::Quaternion q;
          q.setRPY(0, 1.57, 0); 
          target_pose.orientation = tf2::toMsg(q);

          arm_interface.setPoseTarget(target_pose);
          
          auto error_code = arm_interface.move();
          
          if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
              std::cout << "✅ Επιτυχία!" << std::endl;
          } else {
              std::cout << "❌ Αποτυχία σχεδιασμού." << std::endl;
          }

      } catch (const tf2::TransformException & ex) {
          std::cout << "⚠️ Δεν βλέπω το Marker " << target_id << " (ή χάθηκε η Άγκυρα)!" << std::endl;
      }
  }

  rclcpp::shutdown();
  return 0;
}
