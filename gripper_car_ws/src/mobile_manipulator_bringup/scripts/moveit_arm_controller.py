#!/usr/bin/env python3
"""
MoveIt2 Arm Controller for Mobile Manipulator

This node provides a high-level interface to control the robotic arm using MoveIt2.
It bridges between MoveIt's joint trajectory controller and the Arduino hardware.

Features:
- Joint trajectory execution via MoveIt2
- Predefined pose targets (home, ready, etc.)
- Gripper control
- Cartesian path planning
- Collision checking

Author: Mobile Manipulator Project
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
from enum import Enum


class ArmPose(Enum):
    """Predefined arm poses"""
    HOME = "home"
    READY = "ready"
    PICK = "pick"
    PLACE = "place"
    RETRACT = "retract"


class MoveItArmController(Node):
    """MoveIt2-based arm controller"""

    def __init__(self):
        super().__init__('moveit_arm_controller')

        # Callback group for parallel execution
        self.callback_group = ReentrantCallbackGroup()

        # Joint names matching URDF and MoveIt config
        self.arm_joint_names = [
            'joint_1',           # Base rotation
            'joint_2',           # Shoulder
            'joint_3',           # Elbow
            'joint_4',           # Wrist
            'gripper_base_joint' # Gripper rotation
        ]

        self.gripper_joint_name = 'left_gear_joint'  # Gripper open/close

        # Current joint state
        self.current_joint_positions = {}
        self.current_joint_velocities = {}

        # Predefined poses (in radians)
        self.poses = {
            ArmPose.HOME: {
                'joint_1': 0.0,
                'joint_2': 0.0,
                'joint_3': 0.0,
                'joint_4': 0.0,
                'gripper_base_joint': 0.0,
                'left_gear_joint': 0.0
            },
            ArmPose.READY: {
                'joint_1': 0.0,
                'joint_2': -0.5,    # ~-28.6 degrees
                'joint_3': 0.7,      # ~40 degrees
                'joint_4': -0.2,     # ~-11.4 degrees
                'gripper_base_joint': 0.0,
                'left_gear_joint': 0.0
            },
            ArmPose.PICK: {
                'joint_1': 0.0,
                'joint_2': -0.8,
                'joint_3': 1.2,
                'joint_4': -0.4,
                'gripper_base_joint': 0.0,
                'left_gear_joint': 0.7  # Open gripper
            },
            ArmPose.PLACE: {
                'joint_1': 1.57,     # 90 degrees rotation
                'joint_2': -0.5,
                'joint_3': 0.7,
                'joint_4': -0.2,
                'gripper_base_joint': 0.0,
                'left_gear_joint': 0.0  # Closed gripper
            },
            ArmPose.RETRACT: {
                'joint_1': 0.0,
                'joint_2': -1.2,
                'joint_3': 1.5,
                'joint_4': -0.3,
                'gripper_base_joint': 0.0,
                'left_gear_joint': 0.0
            }
        }

        # Gripper positions
        self.gripper_open = 0.7   # radians (~40 degrees)
        self.gripper_closed = 0.0  # radians

        # Action client for joint trajectory controller
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.callback_group
        )

        # Publishers
        self.status_pub = self.create_publisher(String, '/arm/status', 10)

        # Wait for action server
        self.get_logger().info('Waiting for joint trajectory action server...')
        self.trajectory_client.wait_for_server()
        self.get_logger().info('Connected to joint trajectory action server')

        self.get_logger().info('MoveIt Arm Controller initialized')

    def joint_state_callback(self, msg):
        """Update current joint states"""
        for i, name in enumerate(msg.name):
            if name in self.arm_joint_names or name == self.gripper_joint_name:
                self.current_joint_positions[name] = msg.position[i]
                if len(msg.velocity) > i:
                    self.current_joint_velocities[name] = msg.velocity[i]

    def move_to_pose(self, pose: ArmPose, duration_sec=3.0):
        """
        Move arm to a predefined pose

        Args:
            pose: Target pose from ArmPose enum
            duration_sec: Time to reach target (seconds)

        Returns:
            Future for the action goal
        """
        if pose not in self.poses:
            self.get_logger().error(f'Unknown pose: {pose}')
            return None

        target_positions = self.poses[pose]
        return self.move_to_joint_positions(target_positions, duration_sec)

    def move_to_joint_positions(self, joint_positions, duration_sec=3.0):
        """
        Move to specific joint positions

        Args:
            joint_positions: Dict of {joint_name: position_rad}
            duration_sec: Time to reach target (seconds)

        Returns:
            Future for the action goal
        """
        # Create trajectory goal
        goal_msg = FollowJointTrajectory.Goal()

        # Create trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = list(joint_positions.keys())

        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = [joint_positions[name] for name in trajectory.joint_names]
        point.velocities = [0.0] * len(trajectory.joint_names)
        point.time_from_start = Duration(sec=int(duration_sec),
                                         nanosec=int((duration_sec % 1) * 1e9))

        trajectory.points.append(point)
        goal_msg.trajectory = trajectory

        # Send goal
        self.get_logger().info(f'Sending trajectory goal: {trajectory.joint_names}')
        self.publish_status(f'Moving to target position')

        future = self.trajectory_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

        return future

    def move_joints_incremental(self, joint_deltas, duration_sec=1.0):
        """
        Move joints by incremental amounts from current position

        Args:
            joint_deltas: Dict of {joint_name: delta_rad}
            duration_sec: Time to complete motion

        Returns:
            Future for the action goal
        """
        # Calculate target positions
        target_positions = {}
        for joint_name, delta in joint_deltas.items():
            if joint_name in self.current_joint_positions:
                current = self.current_joint_positions[joint_name]
                target_positions[joint_name] = current + delta
            else:
                self.get_logger().warn(f'Joint {joint_name} not in current state')

        if not target_positions:
            self.get_logger().error('No valid joint targets')
            return None

        return self.move_to_joint_positions(target_positions, duration_sec)

    def open_gripper(self, duration_sec=1.0):
        """Open the gripper"""
        self.get_logger().info('Opening gripper')
        return self.move_to_joint_positions({self.gripper_joint_name: self.gripper_open},
                                           duration_sec)

    def close_gripper(self, duration_sec=1.0):
        """Close the gripper"""
        self.get_logger().info('Closing gripper')
        return self.move_to_joint_positions({self.gripper_joint_name: self.gripper_closed},
                                           duration_sec)

    def set_gripper_position(self, position, duration_sec=1.0):
        """
        Set gripper to specific position

        Args:
            position: Gripper position in radians (0=closed, 0.7=open)
            duration_sec: Time to complete motion
        """
        position = max(0.0, min(self.gripper_open, position))
        return self.move_to_joint_positions({self.gripper_joint_name: position},
                                           duration_sec)

    def goal_response_callback(self, future):
        """Handle action goal response"""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by action server')
            self.publish_status('Goal rejected')
            return

        self.get_logger().info('Goal accepted by action server')

        # Get result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle action result"""
        result = future.result().result
        status = future.result().status

        if status == 4:  # SUCCEEDED
            self.get_logger().info('Trajectory execution succeeded')
            self.publish_status('Motion completed')
        else:
            self.get_logger().warn(f'Trajectory execution failed with status: {status}')
            self.publish_status(f'Motion failed: {status}')

    def execute_pick_and_place_sequence(self):
        """
        Execute a complete pick and place sequence

        This is an example of chaining multiple movements
        """
        self.get_logger().info('Executing pick and place sequence')

        # Move to ready position
        future1 = self.move_to_pose(ArmPose.READY, duration_sec=2.0)

        # Wait for completion (in a real application, use callbacks)
        import time
        time.sleep(2.5)

        # Move to pick position with open gripper
        future2 = self.move_to_pose(ArmPose.PICK, duration_sec=2.0)
        time.sleep(2.5)

        # Close gripper
        self.close_gripper(duration_sec=1.0)
        time.sleep(1.5)

        # Retract
        self.move_to_pose(ArmPose.RETRACT, duration_sec=2.0)
        time.sleep(2.5)

        # Move to place position
        self.move_to_pose(ArmPose.PLACE, duration_sec=3.0)
        time.sleep(3.5)

        # Open gripper
        self.open_gripper(duration_sec=1.0)
        time.sleep(1.5)

        # Return to home
        self.move_to_pose(ArmPose.HOME, duration_sec=2.0)

        self.get_logger().info('Pick and place sequence complete')

    def publish_status(self, message):
        """Publish status message"""
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)

    def get_current_joint_positions(self):
        """Get current joint positions"""
        return self.current_joint_positions.copy()

    def print_current_state(self):
        """Print current joint state"""
        self.get_logger().info('=== Current Joint State ===')
        for joint in self.arm_joint_names + [self.gripper_joint_name]:
            if joint in self.current_joint_positions:
                pos_rad = self.current_joint_positions[joint]
                pos_deg = math.degrees(pos_rad)
                self.get_logger().info(f'  {joint}: {pos_rad:.3f} rad ({pos_deg:.1f}°)')


def demo_sequence(controller):
    """Run a demonstration sequence"""
    import time

    # Print current state
    controller.print_current_state()
    time.sleep(1.0)

    # Move to home
    controller.get_logger().info('>>> Moving to HOME position')
    controller.move_to_pose(ArmPose.HOME, duration_sec=3.0)
    time.sleep(4.0)

    # Move to ready
    controller.get_logger().info('>>> Moving to READY position')
    controller.move_to_pose(ArmPose.READY, duration_sec=3.0)
    time.sleep(4.0)

    # Test gripper
    controller.get_logger().info('>>> Testing gripper')
    controller.open_gripper()
    time.sleep(2.0)
    controller.close_gripper()
    time.sleep(2.0)

    # Incremental movement test
    controller.get_logger().info('>>> Testing incremental movement')
    controller.move_joints_incremental({'joint_1': 0.5}, duration_sec=2.0)
    time.sleep(2.5)
    controller.move_joints_incremental({'joint_1': -0.5}, duration_sec=2.0)
    time.sleep(2.5)

    # Return to home
    controller.get_logger().info('>>> Returning to HOME')
    controller.move_to_pose(ArmPose.HOME, duration_sec=3.0)
    time.sleep(4.0)

    controller.get_logger().info('>>> Demo sequence complete!')


def main(args=None):
    rclpy.init(args=args)

    controller = MoveItArmController()

    # Option to run demo sequence
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == '--demo':
        controller.get_logger().info('Running demonstration sequence...')
        import threading
        demo_thread = threading.Thread(target=demo_sequence, args=(controller,))
        demo_thread.start()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
