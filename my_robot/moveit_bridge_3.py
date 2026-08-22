import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState

class MoveItBridge(Node):
    def __init__(self):
        super().__init__('moveit_bridge_node')
        self.get_logger().info('--- MoveIt Bridge (Arm + Gripper) Started ---')

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

        # Ονόματα MoveIt
        self.moveit_arm_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        self.moveit_gripper_name = 'endeffector_gripper' # Όπως το λέει το MoveIt
        
        # Ονόματα Driver
        self.driver_arm_names = [
            'joint1_to_base', 'joint2_to_joint1', 'joint3_to_joint2', 
            'joint4_to_joint3', 'joint5_to_joint4', 'joint6_to_joint5', 'joint7_to_joint6'
        ]
        self.driver_gripper_name = 'endeffector_gripper' # Όπως το θέλει ο lan_driver
        
        # Αρχικές θέσεις
        self.current_arm_pos = [0.0] * 7 
        self.current_gripper_pos = 0.0

        # Timer 10Hz
        self.timer = self.create_timer(0.1, self.publish_current_state)

    def publish_current_state(self):
        # Ενώνουμε τα δεδομένα (7 arm + 1 gripper)
        
        # A. Για το MoveIt
        msg_moveit = JointState()
        msg_moveit.header.stamp = self.get_clock().now().to_msg()
        # Λίστα ονομάτων: 7 του χεριού + 1 του gripper
        msg_moveit.name = self.moveit_arm_names + [self.moveit_gripper_name]
        msg_moveit.position = self.current_arm_pos + [self.current_gripper_pos]
        self.moveit_pub.publish(msg_moveit)

        # B. Για τον Driver (Hardware)
        msg_driver = JointState()
        msg_driver.header.stamp = self.get_clock().now().to_msg()
        msg_driver.name = self.driver_arm_names + [self.driver_gripper_name]
        msg_driver.position = self.current_arm_pos + [self.current_gripper_pos]
        self.driver_pub.publish(msg_driver)

    def arm_callback(self, goal_handle):
        self.get_logger().info('Arm Command Received...')
        points = goal_handle.request.trajectory.points
        start_time = time.time()
        
        for point in points:
            self.current_arm_pos = list(point.positions)
            
            time_from_start = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            elapsed = time.time() - start_time
            sleep_time = time_from_start - elapsed
            
            if sleep_time > 0:
                time.sleep(sleep_time)

        goal_handle.succeed()
        return FollowJointTrajectory.Result()

    def gripper_callback(self, goal_handle):
        self.get_logger().info('Gripper Command Received...')
        points = goal_handle.request.trajectory.points
        start_time = time.time()
        
        for point in points:
            # Το gripper έχει μόνο μία άρθρωση, άρα παίρνουμε το positions[0]
            if len(point.positions) > 0:
                self.current_gripper_pos = point.positions[0]
            
            time_from_start = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            elapsed = time.time() - start_time
            sleep_time = time_from_start - elapsed
            
            if sleep_time > 0:
                time.sleep(sleep_time)

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
