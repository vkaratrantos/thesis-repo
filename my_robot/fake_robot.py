import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
import time

class FakeRobot(Node):
    def __init__(self):
        super().__init__('fake_robot_bridge')
        
        self.js_pub = self.create_publisher(JointState, '/fake_joint_states', 10)
        cb_group = ReentrantCallbackGroup()
        self.arm_server = ActionServer(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory', 
            self.execute_callback, callback_group=cb_group)
            
        self.gripper_server = ActionServer(
            self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory', 
            self.execute_callback, callback_group=cb_group)
            
        self.joint_state = JointState()
        self.joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7', 'endeffector_gripper']
        self.joint_state.position = [0.0] * 8 
        
        self.timer = self.create_timer(0.05, self.timer_cb, callback_group=cb_group)
        self.get_logger().info('Το Fake Robot (Multi-Threaded) τρέχει στο /fake_joint_states!')

    def timer_cb(self):
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.js_pub.publish(self.joint_state)

    def execute_callback(self, goal_handle):
        self.get_logger().info('>>> Ελήφθη σχέδιο! Εκτέλεση...')
        trajectory = goal_handle.request.trajectory
        
        for point in trajectory.points:
            for i, name in enumerate(trajectory.joint_names):
                if name in self.joint_state.name:
                    idx = self.joint_state.name.index(name)
                    self.joint_state.position[idx] = point.positions[i]
            
            self.joint_state.header.stamp = self.get_clock().now().to_msg()
            self.js_pub.publish(self.joint_state)
            time.sleep(0.02) 
            
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        self.get_logger().info('>>> Η κίνηση ολοκληρώθηκε!')
        return result

def main():
    rclpy.init()
    node = FakeRobot()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
