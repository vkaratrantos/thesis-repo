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

        # --- THE FIX: Create a Callback Group for Multi-Threading ---
        self.cb_group = ReentrantCallbackGroup()

        # 1. Publishers
        self.moveit_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.driver_pub = self.create_publisher(JointState, '/hardware_joints', 10)

        # 2. Action Servers (Assigned to the Multi-Threaded Group)
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

        # Assigned to the Multi-Threaded Group so it never gets blocked!
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
        
        # Traffic Control (10Hz max) + Highly sensitive Deadzone (0.0001)
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
            
            # Get starting positions for this specific movement segment
            start_positions = list(self.current_pos[:7])
            
            # --- THE FIX: Linear Interpolation Loop ---
            now = time.time()
            segment_duration = target_time - now
            
            # If the waypoint is more than 0.02s away, slice it up!
            if segment_duration > 0.02:
                steps = int(segment_duration / 0.02)
                for step in range(1, steps + 1):
                    fraction = step / float(steps)
                    for i in range(7):
                        self.current_pos[i] = start_positions[i] + fraction * (target_positions[i] - start_positions[i])
                    time.sleep(0.02)
            else:
                # If it's a tiny Cartesian step, just sleep the remainder
                time.sleep(max(0.0, segment_duration))
            
            # Ensure it snaps perfectly to the target at the very end of the waypoint
            for i in range(7):
                self.current_pos[i] = target_positions[i]

        goal_handle.succeed()
        return FollowJointTrajectory.Result()

    def gripper_callback(self, goal_handle):
        # This used to take points[-1] and jump straight to it, discarding every
        # intermediate point and all the timing -- hence "Snap Command". The
        # consequence was that no velocity scaling on the MoveIt side could ever
        # slow the gripper down, because the trajectory was thrown away.
        #
        # Same interpolation as arm_callback, so the gripper now closes at the
        # speed it was planned for.
        points = goal_handle.request.trajectory.points
        if not points:
            goal_handle.succeed()
            return FollowJointTrajectory.Result()

        # The gripper controller drives one joint, but find it by name rather
        # than assuming index 0.
        names = list(goal_handle.request.trajectory.joint_names)
        idx = names.index('endeffector_gripper') if 'endeffector_gripper' in names else 0

        start_time = time.time()

        for point in points:
            target = point.positions[idx]
            time_from_start = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            target_time = start_time + time_from_start

            start_pos = self.current_pos[7]
            segment_duration = target_time - time.time()

            if segment_duration > 0.02:
                steps = int(segment_duration / 0.02)
                for step in range(1, steps + 1):
                    fraction = step / float(steps)
                    self.current_pos[7] = start_pos + fraction * (target - start_pos)
                    time.sleep(0.02)
            else:
                time.sleep(max(0.0, segment_duration))

            self.current_pos[7] = target

        goal_handle.succeed()
        return FollowJointTrajectory.Result()

def main(args=None):
    rclpy.init(args=args)
    node = MoveItBridge()
    
    # --- THE FIX: Boot the node using multiple threads ---
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
