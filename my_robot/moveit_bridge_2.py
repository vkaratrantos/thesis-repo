import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState

class MoveItBridge(Node):
    def __init__(self):
        super().__init__('moveit_bridge_node')
        self.get_logger().info('--- MoveIt Bridge (Dual Channel) Started ---')

        # 1. Publisher για το MoveIt (Καθαρά ονόματα)
        # Αυτό κρατάει το MoveIt χαρούμενο χωρίς Errors
        self.moveit_pub = self.create_publisher(JointState, '/joint_states', 10)

        # 2. Publisher για τον Driver (Hardware ονόματα)
        # Αυτό το στέλνουμε σε ΕΙΔΙΚΟ topic για να μην μπερδεύεται το MoveIt
        self.driver_pub = self.create_publisher(JointState, '/hardware_joints', 10)

        # 3. Action Server (MoveIt Command Receiver)
        self._arm_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory', 
            self.execute_callback
        )
        
        self._gripper_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory',
            self.gripper_callback
        )

        # Ονόματα MoveIt (Standard)
        self.moveit_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        
        # Ονόματα Driver (Custom)
        self.driver_names = [
            'joint1_to_base', 'joint2_to_joint1', 'joint3_to_joint2', 
            'joint4_to_joint3', 'joint5_to_joint4', 'joint6_to_joint5', 'joint7_to_joint6'
        ]
        
        # Αρχική θέση
        self.current_joints = [0.0] * 7 

        # Heartbeat Timer (10Hz)
        self.timer = self.create_timer(0.1, self.publish_current_state)

    def publish_current_state(self):
        # A. Μήνυμα για το MoveIt (Standard Names)
        msg_moveit = JointState()
        msg_moveit.header.stamp = self.get_clock().now().to_msg()
        msg_moveit.name = self.moveit_names
        msg_moveit.position = self.current_joints
        self.moveit_pub.publish(msg_moveit)

        # B. Μήνυμα για τον Driver (Driver Names)
        msg_driver = JointState()
        msg_driver.header.stamp = self.get_clock().now().to_msg()
        msg_driver.name = self.driver_names
        msg_driver.position = self.current_joints
        self.driver_pub.publish(msg_driver)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing Trajectory...')
        points = goal_handle.request.trajectory.points
        start_time = time.time()
        
        for point in points:
            self.current_joints = list(point.positions)
            
            # (To publish γίνεται αυτόματα από τον timer)
            
            time_from_start = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            elapsed = time.time() - start_time
            sleep_time = time_from_start - elapsed
            
            if sleep_time > 0:
                time.sleep(sleep_time)

        goal_handle.succeed()
        return FollowJointTrajectory.Result()

    def gripper_callback(self, goal_handle):
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
