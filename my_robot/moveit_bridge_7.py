import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState

class MoveItBridge(Node):
    def __init__(self):
        super().__init__('moveit_bridge_node')
        self.get_logger().info('--- MoveIt Smart Bridge (Multi-Threaded) Started ---')
        self.cb_group = ReentrantCallbackGroup()
        self.moveit_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.driver_pub = self.create_publisher(JointState, '/hardware_joints', 10)

        self._arm_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory', 
            self.arm_callback,
            callback_group=self.cb_group
        )
        
        self._gripper_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory',
            self.gripper_callback,
            callback_group=self.cb_group
        )

        self.moveit_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7', 'endeffector_gripper']
        
        self.driver_names = [
            'joint1_to_base', 'joint2_to_joint1', 'joint3_to_joint2', 
            'joint4_to_joint3', 'joint5_to_joint4', 'joint6_to_joint5', 'joint7_to_joint6',
            'endeffector_gripper'
        ]
        
        self.current_pos = [0.0] * 8 
        self.last_sent_pos = [999.0] * 8 
        self.last_sent_time = 0.0
        self.timer = self.create_timer(0.02, self.publish_loop, callback_group=self.cb_group)

    def publish_loop(self):
        now = self.get_clock().now().to_msg()
        
        msg_moveit = JointState()
        msg_moveit.header.stamp = now
        msg_moveit.name = self.moveit_names
        msg_moveit.position = self.current_pos
        self.moveit_pub.publish(msg_moveit)

        current_time = time.time()
        time_diff = current_time - self.last_sent_time
        diff = sum([abs(a - b) for a, b in zip(self.current_pos, self.last_sent_pos)])
        
        should_send = False
        
        if time_diff > 0.05: 
            if diff > 0.0001: 
                should_send = True
            elif time_diff > 1.0: 
                should_send = True

        if should_send:
            msg_driver = JointState()
            msg_driver.header.stamp = now
            msg_driver.name = self.driver_names
            msg_driver.position = self.current_pos
            self.driver_pub.publish(msg_driver)
            
            self.last_sent_pos = list(self.current_pos)
            self.last_sent_time = current_time

    def arm_callback(self, goal_handle):
        points = goal_handle.request.trajectory.points
        if not points:
            goal_handle.succeed()
            return FollowJointTrajectory.Result()

        start_time = time.time()
        
        for point in points:
            target_positions = point.positions[:7]
            time_from_start = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            target_time = start_time + time_from_start
            
            start_positions = list(self.current_pos[:7])
            
            now = time.time()
            segment_duration = target_time - now
            
            if segment_duration > 0.02:
                steps = int(segment_duration / 0.02)
                for step in range(1, steps + 1):
                    fraction = step / float(steps)
                    for i in range(7):
                        self.current_pos[i] = start_positions[i] + fraction * (target_positions[i] - start_positions[i])
                    time.sleep(0.02)
            else:
                time.sleep(max(0.0, segment_duration))
            
            for i in range(7):
                self.current_pos[i] = target_positions[i]

        goal_handle.succeed()
        return FollowJointTrajectory.Result()

    def gripper_callback(self, goal_handle):
        points = goal_handle.request.trajectory.points
        if len(points) > 0:
            final_pos = points[-1].positions[0]
            self.current_pos[7] = final_pos
            self.get_logger().info(f'Gripper Snap Command: {final_pos}')

        goal_handle.succeed()
        return FollowJointTrajectory.Result()

def main(args=None):
    rclpy.init(args=args)
    node = MoveItBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
