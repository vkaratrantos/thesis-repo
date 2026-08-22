import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

class MoveItBridge(Node):
    def __init__(self):
        super().__init__('moveit_bridge_node')
        self.get_logger().info('--- MoveIt Bridge (Translator Mode) Started ---')

        # 1. Publisher προς τον LAN Driver
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # 2. Action Servers
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

        # --- Η ΜΕΤΑΦΡΑΣΗ ---
        # Τα ονόματα που χρησιμοποιεί το MoveIt (από το URDF)
        self.moveit_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        
        # Τα ονόματα που περιμένει ο Driver σου (από το ros_lan_driver.py)
        self.driver_names = [
            'joint1_to_base', 
            'joint2_to_joint1', 
            'joint3_to_joint2', 
            'joint4_to_joint3', 
            'joint5_to_joint4', 
            'joint6_to_joint5', 
            'joint7_to_joint6'
        ]
        
        # Αρχική θέση (Home)
        self.current_joints = [0.0] * 7 

        # 3. Heartbeat Timer (10Hz)
        self.timer = self.create_timer(0.1, self.publish_current_state)

    def publish_current_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # ΕΔΩ ΓΙΝΕΤΑΙ ΤΟ ΚΟΛΠΟ:
        # Στέλνουμε τα ονόματα που θέλει ο Driver, όχι αυτά που έχει το MoveIt
        msg.name = self.driver_names 
        msg.position = self.current_joints
        
        self.joint_pub.publish(msg)

    def execute_callback(self, goal_handle):
        self.get_logger().info('MoveIt sent a command -> Translating for Driver...')
        
        trajectory_points = goal_handle.request.trajectory.points
        start_time = time.time()
        
        for point in trajectory_points:
            # Αποθηκεύουμε τη θέση για το heartbeat
            self.current_joints = list(point.positions)
            
            # (Το publish γίνεται αυτόματα από τον timer με τα σωστά ονόματα)
            
            # Χρονισμός
            time_from_start = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            elapsed = time.time() - start_time
            sleep_time = time_from_start - elapsed
            
            if sleep_time > 0:
                time.sleep(sleep_time)

        goal_handle.succeed()
        return FollowJointTrajectory.Result()

    def gripper_callback(self, goal_handle):
        # Αν χρειαστεί gripper, στέλνουμε 'endeffector_gripper'
        # Προς το παρόν απλά επιστρέφουμε success
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
