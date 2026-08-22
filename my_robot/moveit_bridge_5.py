import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState

class MoveItBridge(Node):
    def __init__(self):
        super().__init__('moveit_bridge_node')
        self.get_logger().info('--- MoveIt Smart Bridge (Rate Limited) Started ---')

        # 1. Publishers
        self.moveit_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.driver_pub = self.create_publisher(JointState, '/hardware_joints', 10)

        # 2. Action Servers
        self._arm_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory', 
            self.arm_callback
        )
        
        self._gripper_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory',
            self.gripper_callback
        )

        # Ονόματα
        self.moveit_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7', 'endeffector_gripper']
        
        # Ονόματα που περιμένει ο Driver (όπως στο ros_lan_driver.py)
        self.driver_names = [
            'joint1_to_base', 'joint2_to_joint1', 'joint3_to_joint2', 
            'joint4_to_joint3', 'joint5_to_joint4', 'joint6_to_joint5', 'joint7_to_joint6',
            'endeffector_gripper'
        ]
        
        # Κατάσταση (8 τιμές: 7 για το χέρι + 1 για το gripper)
        self.current_pos = [0.0] * 8 
        
        # Μεταβλητές για τον έλεγχο ροής (Traffic Control)
        self.last_sent_pos = [999.0] * 8 
        self.last_sent_time = 0.0

        # Τρέχουμε γρήγορα (50Hz) για το MoveIt, αλλά φιλτράρουμε για τον Driver
        self.timer = self.create_timer(0.02, self.publish_loop)

    def publish_loop(self):
        now = self.get_clock().now().to_msg()
        
        # 1. Στο MoveIt στέλνουμε ΠΑΝΤΑ (για να μην βγάζει errors)
        msg_moveit = JointState()
        msg_moveit.header.stamp = now
        msg_moveit.name = self.moveit_names
        msg_moveit.position = self.current_pos
        self.moveit_pub.publish(msg_moveit)

        # 2. Στον Driver στέλνουμε ΕΞΥΠΝΑ
        current_time = time.time()
        time_diff = current_time - self.last_sent_time
        
        # Υπολογίζουμε αν άλλαξε κάτι στη θέση
        diff = sum([abs(a - b) for a, b in zip(self.current_pos, self.last_sent_pos)])
        
        should_send = False
        
        # Κανόνας: Στέλνουμε μόνο αν πέρασαν τουλάχιστον 0.05s (20Hz max)
        # Αυτό προστατεύει τη Serial Port από μποτιλιάρισμα.
        if time_diff > 0.05: 
            if diff > 0.001: # Αν υπάρχει κίνηση
                should_send = True
            elif time_diff > 1.0: # Αν πέρασε 1 sec (Heartbeat)
                should_send = True

        if should_send:
            msg_driver = JointState()
            msg_driver.header.stamp = now
            msg_driver.name = self.driver_names
            msg_driver.position = self.current_pos
            self.driver_pub.publish(msg_driver)
            
            # Ενημερώνουμε τη μνήμη
            self.last_sent_pos = list(self.current_pos)
            self.last_sent_time = current_time

    def arm_callback(self, goal_handle):
        # Το χέρι ακολουθεί την τροχιά κανονικά (Interpolation)
        points = goal_handle.request.trajectory.points
        start_time = time.time()
        
        for point in points:
            # Ανανεώνουμε τις 7 πρώτες θέσεις
            for i in range(7):
                self.current_pos[i] = point.positions[i]
            
            time_from_start = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            elapsed = time.time() - start_time
            sleep_time = time_from_start - elapsed
            
            if sleep_time > 0:
                time.sleep(sleep_time)

        goal_handle.succeed()
        return FollowJointTrajectory.Result()

    def gripper_callback(self, goal_handle):
        # Το Gripper πάει ΚΑΤΕΥΘΕΙΑΝ στο στόχο (Instant Snap)
        points = goal_handle.request.trajectory.points
        if len(points) > 0:
            final_pos = points[-1].positions[0]
            
            # Ανανεώνουμε την 8η θέση (Index 7)
            self.current_pos[7] = final_pos
            self.get_logger().info(f'Gripper Snap Command: {final_pos}')

        goal_handle.succeed()
        return FollowJointTrajectory.Result()

def main(args=None):
    rclpy.init(args=args)
    node = MoveItBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
