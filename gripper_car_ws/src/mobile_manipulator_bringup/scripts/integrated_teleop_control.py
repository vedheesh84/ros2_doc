#!/usr/bin/env python3
"""
Integrated Teleop Control for Mobile Manipulator

This script provides keyboard control for:
- Mobile base (via cmd_vel using teleop_twist_keyboard)
- Robotic arm (via MoveIt2 motion planning)

Key Controls:
-----------
MOBILE BASE (always active):
  i/,  - forward/backward
  j/l  - turn left/right
  k    - stop
  q/z  - increase/decrease max speeds
  w/x  - increase/decrease only linear speed
  e/c  - increase/decrease only angular speed

ARM CONTROL (press 'm' to activate):
  1-6  - Select joint (joint_1 to left_gear_joint)
  +/-  - Increase/decrease selected joint angle
  h    - Move arm to home position
  g    - Open gripper
  f    - Close gripper
  p    - Plan to current target
  e    - Execute planned trajectory
  s    - Stop arm motion
  m    - Toggle arm control mode

Author: Mobile Manipulator Project
"""

import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import threading
import math

# Try to import MoveIt2 python interface
try:
    from moveit_msgs.msg import DisplayTrajectory
    from moveit_msgs.srv import GetPositionIK
    MOVEIT_AVAILABLE = True
except ImportError:
    MOVEIT_AVAILABLE = False
    print("Warning: MoveIt2 not available. Arm control will use direct joint commands.")


class IntegratedTeleopControl(Node):
    """Integrated teleop control for mobile base and arm"""

    def __init__(self):
        super().__init__('integrated_teleop_control')

        # Control mode
        self.arm_control_mode = False  # False = base, True = arm

        # Mobile base parameters
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.speed_linear = 0.5    # m/s
        self.speed_angular = 1.0   # rad/s
        self.speed_increment = 0.1

        # Arm parameters
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4',
                           'gripper_base_joint', 'left_gear_joint']
        self.current_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # radians
        self.target_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.selected_joint = 0
        self.joint_increment = 0.1  # radians

        # Home and preset positions (radians)
        self.home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.gripper_open_angle = 0.7  # radians (~40 degrees)
        self.gripper_close_angle = 0.0

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_cmd_pub = self.create_publisher(JointState, '/arm/joint_commands', 10)
        self.status_pub = self.create_publisher(String, '/teleop/status', 10)

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

        # Timer for publishing commands
        self.timer = self.create_timer(0.1, self.publish_commands)

        # Terminal settings
        self.settings = None

        self.get_logger().info('Integrated Teleop Control Started')
        self.print_usage()

    def print_usage(self):
        """Print control instructions"""
        msg = """
╔═══════════════════════════════════════════════════════════════════╗
║         INTEGRATED MOBILE MANIPULATOR TELEOP CONTROL              ║
╠═══════════════════════════════════════════════════════════════════╣
║ MOBILE BASE CONTROL (Default Mode):                              ║
║   i / ,     - Move forward / backward                             ║
║   j / l     - Turn left / right                                   ║
║   k         - Stop all motion                                     ║
║   q / z     - Increase / decrease max speeds                      ║
║   w / x     - Increase / decrease linear speed only               ║
║   e / c     - Increase / decrease angular speed only              ║
║                                                                   ║
║ ARM CONTROL MODE (Press 'M' to activate):                        ║
║   1-6       - Select joint (1=base, 2=shoulder, 3=elbow,         ║
║               4=wrist, 5=gripper_base, 6=gripper_open/close)     ║
║   + / -     - Increase / decrease selected joint angle           ║
║   [ / ]     - Larger increment/decrement (0.5 rad)               ║
║   h         - Move arm to HOME position                           ║
║   g         - Open gripper                                        ║
║   f         - Close gripper                                       ║
║   s         - Stop arm motion                                     ║
║                                                                   ║
║ MODE SWITCHING:                                                   ║
║   m         - Toggle between BASE and ARM control                 ║
║                                                                   ║
║ GENERAL:                                                          ║
║   SPACE     - Emergency stop (base and arm)                       ║
║   ESC       - Exit teleop                                         ║
╚═══════════════════════════════════════════════════════════════════╝
"""
        print(msg)

    def joint_state_callback(self, msg):
        """Update current joint positions from /joint_states"""
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_joint_positions[i] = msg.position[idx]

    def publish_commands(self):
        """Publish velocity and joint commands at fixed rate"""
        # Publish base velocity
        twist = Twist()
        twist.linear.x = self.linear_velocity
        twist.angular.z = self.angular_velocity
        self.cmd_vel_pub.publish(twist)

        # Publish arm joint commands if in arm mode
        if self.arm_control_mode:
            joint_cmd = JointState()
            joint_cmd.header.stamp = self.get_clock().now().to_msg()
            joint_cmd.name = self.joint_names
            joint_cmd.position = self.target_joint_positions
            self.joint_cmd_pub.publish(joint_cmd)

    def get_key(self, timeout=0.1):
        """Get keyboard input with timeout"""
        if select.select([sys.stdin], [], [], timeout)[0]:
            return sys.stdin.read(1)
        return None

    def stop_base(self):
        """Stop mobile base"""
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.target_linear = 0.0
        self.target_angular = 0.0

    def stop_arm(self):
        """Stop arm motion"""
        self.target_joint_positions = self.current_joint_positions.copy()

    def emergency_stop(self):
        """Emergency stop everything"""
        self.stop_base()
        self.stop_arm()
        status_msg = String()
        status_msg.data = "EMERGENCY STOP"
        self.status_pub.publish(status_msg)
        self.get_logger().warn('⚠ EMERGENCY STOP ACTIVATED')

    def move_arm_to_home(self):
        """Move arm to home position"""
        self.target_joint_positions = self.home_position.copy()
        self.get_logger().info('Moving arm to HOME position')

    def open_gripper(self):
        """Open gripper"""
        self.target_joint_positions[5] = self.gripper_open_angle
        self.get_logger().info('Opening gripper')

    def close_gripper(self):
        """Close gripper"""
        self.target_joint_positions[5] = self.gripper_close_angle
        self.get_logger().info('Closing gripper')

    def increment_joint(self, amount):
        """Increment selected joint"""
        self.target_joint_positions[self.selected_joint] += amount
        # Clamp to reasonable limits
        self.target_joint_positions[self.selected_joint] = max(
            -math.pi, min(math.pi, self.target_joint_positions[self.selected_joint]))

    def print_status(self):
        """Print current status"""
        mode_str = "ARM CONTROL" if self.arm_control_mode else "BASE CONTROL"

        if self.arm_control_mode:
            joint_name = self.joint_names[self.selected_joint]
            joint_angle = math.degrees(self.target_joint_positions[self.selected_joint])
            print(f"\r[{mode_str}] Selected: {joint_name} | "
                  f"Target: {joint_angle:.1f}° | "
                  f"Current: {math.degrees(self.current_joint_positions[self.selected_joint]):.1f}°",
                  end='', flush=True)
        else:
            print(f"\r[{mode_str}] Linear: {self.linear_velocity:.2f} m/s | "
                  f"Angular: {self.angular_velocity:.2f} rad/s | "
                  f"Max: {self.speed_linear:.2f}/{self.speed_angular:.2f}",
                  end='', flush=True)

    def run(self):
        """Main control loop"""
        self.settings = termios.tcgetattr(sys.stdin)

        try:
            tty.setcbreak(sys.stdin.fileno())

            while rclpy.ok():
                key = self.get_key()

                if key:
                    # Exit condition
                    if key == '\x1b':  # ESC
                        break

                    # Emergency stop
                    elif key == ' ':
                        self.emergency_stop()

                    # Mode switch
                    elif key.lower() == 'm':
                        self.arm_control_mode = not self.arm_control_mode
                        mode = "ARM" if self.arm_control_mode else "BASE"
                        print(f"\n>>> Switched to {mode} CONTROL MODE <<<")

                    # Base control commands
                    elif not self.arm_control_mode:
                        if key == 'i':
                            self.linear_velocity = self.speed_linear
                        elif key == ',':
                            self.linear_velocity = -self.speed_linear
                        elif key == 'j':
                            self.angular_velocity = self.speed_angular
                        elif key == 'l':
                            self.angular_velocity = -self.speed_angular
                        elif key == 'k':
                            self.stop_base()
                        elif key == 'q':
                            self.speed_linear += self.speed_increment
                            self.speed_angular += self.speed_increment
                        elif key == 'z':
                            self.speed_linear = max(0.1, self.speed_linear - self.speed_increment)
                            self.speed_angular = max(0.1, self.speed_angular - self.speed_increment)
                        elif key == 'w':
                            self.speed_linear += self.speed_increment
                        elif key == 'x':
                            self.speed_linear = max(0.1, self.speed_linear - self.speed_increment)
                        elif key == 'e':
                            self.speed_angular += self.speed_increment
                        elif key == 'c':
                            self.speed_angular = max(0.1, self.speed_angular - self.speed_increment)

                    # Arm control commands
                    elif self.arm_control_mode:
                        if key in '123456':
                            self.selected_joint = int(key) - 1
                            print(f"\nSelected joint: {self.joint_names[self.selected_joint]}")
                        elif key == '+' or key == '=':
                            self.increment_joint(self.joint_increment)
                        elif key == '-' or key == '_':
                            self.increment_joint(-self.joint_increment)
                        elif key == '[':
                            self.increment_joint(-0.5)  # Larger decrement
                        elif key == ']':
                            self.increment_joint(0.5)   # Larger increment
                        elif key.lower() == 'h':
                            self.move_arm_to_home()
                        elif key.lower() == 'g':
                            self.open_gripper()
                        elif key.lower() == 'f':
                            self.close_gripper()
                        elif key.lower() == 's':
                            self.stop_arm()

                    self.print_status()

        except Exception as e:
            self.get_logger().error(f'Error in control loop: {e}')

        finally:
            # Stop everything before exit
            self.emergency_stop()

            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

            print("\n\nTeleop control terminated.")


def main(args=None):
    rclpy.init(args=args)

    node = IntegratedTeleopControl()

    # Run in separate thread to allow keyboard input
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
