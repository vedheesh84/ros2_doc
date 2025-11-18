# Mobile Manipulator Integration Guide

Complete guide for the unified four-wheel mobile base + 6-DOF arm manipulator system.

## 📋 Table of Contents

1. [Hardware Setup](#hardware-setup)
2. [Arduino Setup](#arduino-setup)
3. [ROS2 Setup](#ros2-setup)
4. [Running the System](#running-the-system)
5. [Control Interfaces](#control-interfaces)
6. [Troubleshooting](#troubleshooting)

---

## 🔧 Hardware Setup

### Components Required

| Component | Quantity | Description |
|-----------|----------|-------------|
| Arduino Mega 2560 | 1 | Main controller |
| L298N Motor Driver | 2 | For 4 DC motors |
| PCA9685 Servo Driver | 1 | For 6 servos (I2C) |
| DC Motors with Encoders | 4 | Quadrature encoders |
| Servo Motors | 6 | 5 for arm + 1 for gripper |
| Power Supply | 2 | 12V for motors, 5V for servos |
| USB Cable | 1 | Arduino to PC connection |

### Wiring Diagram

#### Motor Drivers (L298N x2)

**Driver 1 - Back Motors:**
```
Back Left Motor (BL):
  - IN1  → Arduino Pin 2
  - IN2  → Arduino Pin 3
  - EN   → Arduino Pin 10 (PWM)
  - Motor → M1 terminals

Back Right Motor (BR):
  - IN1  → Arduino Pin 4
  - IN2  → Arduino Pin 5
  - EN   → Arduino Pin 11 (PWM)
  - Motor → M2 terminals

Power:
  - 12V  → Connect to 12V battery
  - GND  → Common ground
  - 5V   → Can power Arduino (optional)
```

**Driver 2 - Front Motors:**
```
Front Left Motor (FL):
  - IN1  → Arduino Pin 6
  - IN2  → Arduino Pin 7
  - EN   → Arduino Pin 12 (PWM)
  - Motor → M3 terminals

Front Right Motor (FR):
  - IN1  → Arduino Pin 8
  - IN2  → Arduino Pin 9
  - EN   → Arduino Pin 13 (PWM)
  - Motor → M4 terminals

Power:
  - 12V  → Connect to 12V battery
  - GND  → Common ground
```

#### Encoders (Quadrature)

```
Back Left (BL):
  - Channel A → Pin 18 (INT5)
  - Channel B → Pin 19 (INT4)

Back Right (BR):
  - Channel A → Pin 20 (INT3)
  - Channel B → Pin 21 (INT2)

Front Left (FL):
  - Channel A → Pin 2 (INT0)  * Note: May need reassignment
  - Channel B → Pin 22

Front Right (FR):
  - Channel A → Pin 3 (INT1)  * Note: May need reassignment
  - Channel B → Pin 23

Power: 5V from Arduino
```

**⚠️ IMPORTANT:** Pins 2 and 3 are shared between motor control and encoders in the current setup. You may need to reassign motor control pins (e.g., move to pins 24-29) to avoid conflicts.

#### PCA9685 Servo Driver

```
I2C Connection:
  - SDA → Arduino A4 (SDA)
  - SCL → Arduino A5 (SCL)
  - VCC → 5V
  - GND → Common ground

Servo Channels:
  - Channel 0 → joint_1 (Base rotation)
  - Channel 1 → joint_2 (Shoulder)
  - Channel 2 → joint_3 (Elbow)
  - Channel 4 → joint_4 (Wrist)
  - Channel 5 → gripper_base_joint (Gripper rotation)
  - Channel 6 → left_gear_joint (Gripper open/close)

Power:
  - V+ → 5-6V servo power supply (separate from Arduino)
  - GND → Common ground (all grounds connected)
```

### Power Distribution

```
┌─────────────────┐
│  12V Battery    │
│  (Mobile Base)  │
└────────┬────────┘
         │
    ┌────┴─────┬──────────┐
    │          │          │
┌───▼───┐  ┌───▼───┐  ┌───▼────┐
│L298N 1│  │L298N 2│  │ BEC/   │
│       │  │       │  │ Buck   │ 5V
└───────┘  └───────┘  │Convert │───┐
                      └────────┘   │
                                   │
┌─────────────────┐                │
│  5-6V Battery   │                │
│  (Servos)       │                │
└────────┬────────┘                │
         │                         │
    ┌────┴────┐               ┌────▼────┐
    │ PCA9685 │               │ Arduino │
    │ (Servos)│◄──────I2C─────┤  Mega   │
    └─────────┘               └─────────┘
         │
    (6x Servos)

⚠️ CRITICAL: Use separate power for servos to avoid Arduino brown-out!
```

---

## 💾 Arduino Setup

### 1. Install Arduino IDE

Download from: https://www.arduino.cc/en/software

### 2. Install Required Libraries

In Arduino IDE, go to **Sketch → Include Library → Manage Libraries**, then install:

- **Adafruit PWM Servo Driver Library**
  - Search: "Adafruit PWM Servo"
  - Install: "Adafruit PWM Servo Driver Library" by Adafruit

### 3. Upload Firmware

1. Open the Arduino sketch:
   ```
   gripper_car_ws/src/mobile_manipulator_bringup/arduino/unified_mobile_manipulator_mega.ino
   ```

2. Select board:
   - **Tools → Board → Arduino AVR Boards → Arduino Mega or Mega 2560**

3. Select processor:
   - **Tools → Processor → ATmega2560 (Mega 2560)**

4. Select port:
   - **Tools → Port → /dev/ttyACM0** (or appropriate port)

5. Click **Upload** (→ button)

6. Verify in Serial Monitor (115200 baud):
   ```
   STATUS,Unified Mobile Manipulator Initializing
   STATUS,Motors Initialized
   STATUS,Encoders Initialized
   STATUS,Servos Initialized
   STATUS,Initialization Complete
   STATUS,Ready for Commands
   ```

### 4. Test Hardware

#### Test Motors
Send via Serial Monitor (115200 baud, Newline ending):
```
VEL,0.2,0.0      # Move forward slowly
VEL,0.0,0.5      # Rotate in place
STOP             # Emergency stop
```

#### Test Servos
```
SERVO,0,90       # Center joint_1
SERVO,5,45       # Open gripper halfway
SET_ALL_SERVOS,90,90,90,90,90,0    # All center, gripper closed
```

#### Test Encoders
Motor movement should produce:
```
ODOM,1234,5678,9012,3456,12345
ODOM,1240,5685,9020,3463,12355
...
```

---

## 🚀 ROS2 Setup

### 1. Build Workspace

```bash
cd ~/ros2_doc/gripper_car_ws
colcon build --packages-select mobile_manipulator_bringup
source install/setup.bash
```

### 2. Configure Serial Port

Find your Arduino:
```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
```

Create udev rule for persistent name (optional but recommended):
```bash
# Get device info
udevadm info --name=/dev/ttyACM0 | grep SERIAL

# Create rule
sudo nano /etc/udev/rules.d/99-mobile-manipulator.rules
```

Add:
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0042", SYMLINK+="mobile_manipulator"
```

Reload:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Give permissions:
```bash
sudo usermod -a -G dialout $USER
# Log out and back in for this to take effect
```

### 3. Test Hardware Bridge

```bash
ros2 run mobile_manipulator_bringup unified_hardware_bridge.py \
  --ros-args -p serial_port:=/dev/ttyACM0
```

Check topics:
```bash
# In another terminal
ros2 topic list
# Should see:
#   /odom
#   /joint_states
#   /hardware/status
#   /cmd_vel
#   /arm/joint_commands

ros2 topic echo /odom
ros2 topic echo /joint_states
```

---

## 🎮 Running the System

### Quick Start - Full System

Single command to launch everything:

```bash
ros2 launch mobile_manipulator_bringup complete_system.launch.py
```

This starts:
- ✅ Hardware bridge (Arduino communication)
- ✅ Robot state publisher (URDF/TF)
- ✅ RViz visualization
- ✅ Integrated teleop (keyboard control)
- ✅ MoveIt arm controller

### Individual Components

#### 1. Hardware Only

```bash
ros2 launch mobile_manipulator_bringup hardware_bringup.launch.py \
  serial_port:=/dev/ttyACM0
```

#### 2. Teleop Control

```bash
# Integrated teleop (base + arm)
ros2 launch mobile_manipulator_bringup teleop_control.launch.py

# Or run directly:
ros2 run mobile_manipulator_bringup integrated_teleop_control.py
```

#### 3. MoveIt Arm Control

```bash
ros2 launch mobile_manipulator_bringup moveit_control.launch.py

# With demo sequence:
ros2 launch mobile_manipulator_bringup moveit_control.launch.py demo:=true
```

### Custom Launch Examples

```bash
# No RViz (headless)
ros2 launch mobile_manipulator_bringup complete_system.launch.py \
  use_rviz:=false

# No MoveIt (teleop only)
ros2 launch mobile_manipulator_bringup complete_system.launch.py \
  use_moveit:=false

# Custom serial port
ros2 launch mobile_manipulator_bringup complete_system.launch.py \
  serial_port:=/dev/ttyUSB0 baud_rate:=115200

# Different wheel parameters
ros2 launch mobile_manipulator_bringup complete_system.launch.py \
  wheel_radius:=0.08 wheel_base:=0.7 encoder_ticks_per_rev:=400
```

---

## 🎯 Control Interfaces

### 1. Integrated Teleop Control

**Press 'm' to toggle between BASE and ARM control modes.**

#### BASE CONTROL MODE (Default)

```
┌─────────────────────────────────────────┐
│  i / ,   - Move forward / backward      │
│  j / l   - Turn left / right            │
│  k       - Stop                         │
│  q / z   - Increase / decrease speeds   │
│  SPACE   - Emergency stop               │
└─────────────────────────────────────────┘
```

Example:
- Press `i` to move forward
- Press `j` to turn left while moving
- Press `k` to stop
- Press `q` several times to increase max speed

#### ARM CONTROL MODE (Press 'm')

```
┌─────────────────────────────────────────┐
│  1-6     - Select joint                 │
│           1: base, 2: shoulder          │
│           3: elbow, 4: wrist            │
│           5: gripper_base, 6: gripper   │
│  +/-     - Increment / decrement joint  │
│  [/]     - Large increment / decrement  │
│  h       - Move to HOME position        │
│  g       - Open gripper                 │
│  f       - Close gripper                │
│  s       - Stop arm motion              │
└─────────────────────────────────────────┘
```

Example workflow:
1. Press `m` to switch to ARM mode
2. Press `1` to select joint_1 (base)
3. Press `+` several times to rotate base
4. Press `6` to select gripper
5. Press `g` to open, `f` to close
6. Press `h` to return to home
7. Press `m` to return to BASE mode

### 2. Direct Topic Control

#### Mobile Base (cmd_vel)

```bash
# Move forward at 0.3 m/s
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Rotate in place at 0.5 rad/s
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"

# Stop
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

#### Arm Control (joint_commands)

```bash
# Move joints to specific positions (radians)
ros2 topic pub /arm/joint_commands sensor_msgs/JointState \
  "{name: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'gripper_base_joint', 'left_gear_joint'], \
    position: [0.0, -0.5, 0.7, -0.2, 0.0, 0.7]}"

# Open gripper
ros2 topic pub /arm/joint_commands sensor_msgs/JointState \
  "{name: ['left_gear_joint'], position: [0.7]}"

# Close gripper
ros2 topic pub /arm/joint_commands sensor_msgs/JointState \
  "{name: ['left_gear_joint'], position: [0.0]}"
```

### 3. MoveIt Python Interface

```python
#!/usr/bin/env python3
import rclpy
from moveit_arm_controller import MoveItArmController, ArmPose

rclpy.init()
controller = MoveItArmController()

# Move to predefined poses
controller.move_to_pose(ArmPose.HOME)
controller.move_to_pose(ArmPose.READY)

# Control gripper
controller.open_gripper()
controller.close_gripper()

# Incremental movement
controller.move_joints_incremental({'joint_1': 0.5})  # Rotate base 0.5 rad

# Custom position
controller.move_to_joint_positions({
    'joint_1': 0.0,
    'joint_2': -0.5,
    'joint_3': 0.7,
    'joint_4': -0.2
})

rclpy.shutdown()
```

### 4. Standard teleop_twist_keyboard

If you prefer the standard ROS2 teleop:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Controls:
```
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
```

---

## 📊 Monitoring and Debugging

### View Active Topics

```bash
ros2 topic list
```

Expected output:
```
/cmd_vel
/odom
/joint_states
/arm/joint_commands
/arm/status
/hardware/status
/tf
/tf_static
/parameter_events
/rosout
```

### Monitor Odometry

```bash
ros2 topic echo /odom
```

### Monitor Joint States

```bash
ros2 topic echo /joint_states
```

### View TF Tree

```bash
ros2 run tf2_tools view_frames
evince frames.pdf
```

### Check Node Status

```bash
ros2 node list
ros2 node info /unified_hardware_bridge
```

### RViz Visualization

RViz should show:
- Robot model (mobile base + arm)
- TF frames (odom → base_link → arm links)
- Odometry path
- Joint states visualization

Add displays:
1. **RobotModel** - Shows URDF
2. **TF** - Shows coordinate frames
3. **Odometry** - Shows /odom with covariance
4. **Path** - Shows trajectory history

---

## 🐛 Troubleshooting

### Arduino Not Connecting

**Symptom:** `Failed to connect: [Errno 2] could not open port /dev/ttyACM0`

**Solutions:**
```bash
# Check if device exists
ls -l /dev/ttyACM*

# Check permissions
sudo chmod 666 /dev/ttyACM0

# Add user to dialout group
sudo usermod -a -G dialout $USER
# Log out and back in

# Check if another process is using it
lsof /dev/ttyACM0
```

### Motors Not Moving

**Symptom:** No motor response to commands

**Checks:**
1. Verify L298N power (12V connected, LED on)
2. Test motor drivers directly:
   ```bash
   # Via Arduino Serial Monitor
   VEL,0.2,0.0
   ```
3. Check motor driver connections
4. Verify enable pins (10, 11, 12, 13) are PWM capable
5. Check if emergency stop is active

### Servos Not Moving

**Symptom:** Arm doesn't respond

**Checks:**
1. Verify PCA9685 power (5V, separate from Arduino if possible)
2. Check I2C connection (A4=SDA, A5=SCL)
3. Test I2C address:
   ```bash
   # Arduino Serial Monitor
   GET_STATUS
   ```
4. Check servo channel wiring (0-6)
5. Verify servo power supply (servos can draw high current)

### Encoder Issues

**Symptom:** Odometry drifts or doesn't update

**Checks:**
1. Monitor encoder output:
   ```bash
   ros2 topic echo /odom
   ```
2. Check encoder wiring (A and B channels)
3. Verify interrupt pins are correct
4. Test encoder manually (spin wheel, check counts):
   ```bash
   # Arduino Serial Monitor should show ODOM messages
   ```

### ROS2 Bridge Not Publishing

**Symptom:** Topics exist but no data

**Checks:**
```bash
# Check node is running
ros2 node list | grep hardware_bridge

# Check for errors
ros2 node info /unified_hardware_bridge

# Restart bridge
ros2 run mobile_manipulator_bringup unified_hardware_bridge.py \
  --ros-args -p serial_port:=/dev/ttyACM0 --log-level debug
```

### MoveIt Not Planning

**Symptom:** MoveIt fails to plan paths

**Checks:**
1. Verify joint limits in URDF
2. Check MoveIt configuration:
   ```bash
   ros2 launch mobile_manipulator_bringup moveit_control.launch.py
   ```
3. Ensure joint_states topic is publishing
4. Check for collisions in RViz
5. Verify controller manager is running

### Communication Timeout

**Symptom:** `Serial read error` or connection drops

**Solutions:**
1. Check USB cable quality
2. Verify baud rate matches (115200)
3. Add ferrite bead to USB cable
4. Try different USB port
5. Check for electrical noise from motors

### Pin Conflicts (Encoders vs Motors)

**Symptom:** Encoders or motors behave erratically

**Solution:** The current firmware uses pins 2 and 3 for both motor control and encoder interrupts. Reassign motor pins:

In `unified_mobile_manipulator_mega.ino`, change:
```cpp
// OLD (conflict):
#define BL_IN1  2   // Conflicts with FL_ENC_A
#define BL_IN2  3   // Conflicts with FR_ENC_A

// NEW (no conflict):
#define BL_IN1  24
#define BL_IN2  25
```

---

## 📐 Calibration

### Wheel Odometry Calibration

1. Mark starting position
2. Command robot to move 1 meter forward:
   ```bash
   ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
     "{linear: {x: 0.2, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}"
   # Let it run for exactly 5 seconds, then stop
   ```
3. Measure actual distance traveled
4. Adjust `wheel_radius` in launch file:
   ```
   actual_distance / commanded_distance = correction_factor
   new_wheel_radius = old_wheel_radius * correction_factor
   ```

### Rotation Calibration

1. Mark starting orientation
2. Command 360° rotation:
   ```bash
   # For exactly 2π radians at 0.5 rad/s = ~12.57 seconds
   ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
     "{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.5}}"
   ```
3. Measure actual rotation
4. Adjust `wheel_base` parameter

### Servo Calibration

For each servo, adjust pulse width mapping in Arduino code:
```cpp
// In unified_mobile_manipulator_mega.ino
#define SERVO_MIN_PULSE 500   // Adjust if servo doesn't reach full range
#define SERVO_MAX_PULSE 2500  // Adjust if servo doesn't reach full range
```

Test with Serial Monitor:
```
SERVO,0,0     # Minimum position
SERVO,0,90    # Center
SERVO,0,180   # Maximum
```

---

## 🔒 Safety Considerations

1. **Emergency Stop:**
   - Press SPACE in teleop to stop everything
   - Arduino has 300ms watchdog timeout
   - Physical E-stop button can be connected to pin 2

2. **Power Safety:**
   - Use separate power for motors (12V) and servos (5-6V)
   - Install fuses on power lines
   - Monitor battery voltage (low voltage = erratic behavior)

3. **Motion Limits:**
   - Software joint limits defined in URDF
   - Hardware limits in Arduino (0-180° for servos)
   - Velocity limits in controllers

4. **Collision Avoidance:**
   - MoveIt provides self-collision checking
   - Add collision objects in RViz for environment

---

## 📚 Additional Resources

- ROS2 Documentation: https://docs.ros.org/en/humble/
- MoveIt2 Tutorials: https://moveit.picknik.ai/humble/
- Arduino Reference: https://www.arduino.cc/reference/en/
- PCA9685 Datasheet: https://www.adafruit.com/product/815

---

## 🎓 Example Applications

### 1. Simple Pick and Place

```python
#!/usr/bin/env python3
import rclpy
from moveit_arm_controller import MoveItArmController, ArmPose

rclpy.init()
controller = MoveItArmController()

# Home position
controller.move_to_pose(ArmPose.HOME)

# Approach object
controller.move_to_pose(ArmPose.PICK)

# Grasp
controller.close_gripper()

# Lift
controller.move_to_pose(ArmPose.RETRACT)

# Move to place location
controller.move_to_pose(ArmPose.PLACE)

# Release
controller.open_gripper()

# Return home
controller.move_to_pose(ArmPose.HOME)

rclpy.shutdown()
```

### 2. Autonomous Navigation + Manipulation

Combine with Nav2 for mobile manipulation tasks:
```bash
# Terminal 1: Hardware
ros2 launch mobile_manipulator_bringup complete_system.launch.py \
  use_nav:=true

# Terminal 2: Set navigation goal
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "..."

# Terminal 3: When arrived, manipulate
ros2 run mobile_manipulator_bringup moveit_arm_controller.py --demo
```

---

## 📝 Notes

- Always test in open space first
- Start with low speeds and gradually increase
- Monitor battery levels
- Keep emergency stop accessible
- Document any hardware modifications

**For support or questions, refer to the main repository documentation.**

---

*Last updated: 2025-01*
*Version: 1.0.0*
