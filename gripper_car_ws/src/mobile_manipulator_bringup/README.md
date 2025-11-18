# Mobile Manipulator Bringup Package

**Complete mobile manipulator system combining 4-wheel differential drive base with 6-DOF robotic arm**

## 📦 Package Overview

This package provides a fully integrated mobile manipulation platform for both **simulation (Gazebo)** and **real hardware** deployment.

### System Components

1. **Mobile Base**: 4-wheel differential drive platform
   - L298N motor drivers
   - Quadrature encoders for odometry
   - SLAM and Nav2 navigation capable

2. **Robotic Arm**: 6-DOF manipulator
   - 5 arm joints + 1 gripper
   - PCA9685 I2C servo controller
   - MoveIt2 motion planning

3. **Sensors**:
   - Lidar (360° laser scanner)
   - RGB camera
   - Optional: IMU, depth camera

4. **Control**: Single Arduino Uno
   - Unified firmware for base + arm
   - Real-time encoder odometry
   - Smooth servo interpolation

---

## 🏗️ Package Structure

```
mobile_manipulator_bringup/
├── arduino/
│   └── unified_mobile_manipulator.ino    # Single Arduino firmware
├── scripts/
│   └── unified_hardware_bridge.py        # ROS2 ↔ Arduino bridge
├── urdf/
│   ├── mobile_base.urdf.xacro           # Modular base description
│   ├── mobile_arm.urdf.xacro            # Modular arm description
│   └── mobile_manipulator.urdf.xacro    # Integrated robot
├── launch/
│   ├── sim/                             # Simulation launches
│   ├── hardware/                        # Hardware launches
│   └── bringup.launch.py                # Master launch file
├── config/
│   ├── base/                            # Base configs (Nav2, SLAM)
│   ├── arm/                             # Arm configs (MoveIt)
│   └── hardware/                        # Hardware parameters
├── meshes/                              # STL/DAE mesh files
├── worlds/                              # Gazebo world files
└── README.md                            # This file
```

---

## 🚀 Quick Start

### Prerequisites

```bash
# ROS2 Humble + required packages
sudo apt install ros-humble-desktop
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-moveit
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers

# Python dependencies
pip3 install pyserial numpy

# Arduino libraries (via Arduino IDE Library Manager)
# - Adafruit PWM Servo Driver Library
# - Wire (included with Arduino)
```

### Building

```bash
cd ~/ros2_doc/gripper_car_ws
colcon build --packages-select mobile_manipulator_bringup
source install/setup.bash
```

---

## 🎮 Usage Modes

### Mode 1: Full Simulation (Gazebo + RViz + MoveIt + Nav2)

```bash
# Launch everything in simulation
ros2 launch mobile_manipulator_bringup bringup.launch.py mode:=simulation

# Or step-by-step:
# 1. Gazebo + RViz
ros2 launch mobile_manipulator_bringup sim/gazebo.launch.py

# 2. MoveIt (in new terminal)
ros2 launch mobile_manipulator_bringup sim/moveit.launch.py

# 3. Navigation (in new terminal)
ros2 launch mobile_manipulator_bringup sim/navigation.launch.py
```

### Mode 2: Real Hardware - Complete System

```bash
# Full hardware bringup (base + arm + nav + moveit)
ros2 launch mobile_manipulator_bringup bringup.launch.py \
  mode:=hardware \
  serial_port:=/dev/ttyACM0

# Keyboard teleoperation
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Mode 3: Base Only (Navigation and SLAM)

```bash
# Hardware base with navigation
ros2 launch mobile_manipulator_bringup bringup.launch.py \
  mode:=base_only \
  serial_port:=/dev/ttyACM0

# SLAM mapping
ros2 launch mobile_manipulator_bringup base_mapping.launch.py
```

### Mode 4: Arm Only (MoveIt Planning)

```bash
# Hardware arm with MoveIt
ros2 launch mobile_manipulator_bringup bringup.launch.py \
  mode:=arm_only \
  serial_port:=/dev/ttyACM0

# MoveIt RViz interface will launch automatically
```

---

## 🔌 Hardware Setup

### Arduino Wiring

**Single Arduino Uno controlling everything:**

#### L298N Motor Driver (Mobile Base)
```
Motor 1 (Back Left):   IN1=D2, IN2=D3
Motor 2 (Back Right):  IN1=D4, IN2=D5
Motor 3 (Front Left):  IN1=D6, IN2=D7
Motor 4 (Front Right): IN1=D8, IN2=D9
Enable Left (M1+M3):   ENA=D10 (PWM)
Enable Right (M2+M4):  ENB=D11 (PWM)
```

#### Quadrature Encoders (Odometry)
```
Encoder M1: A=D18 (INT5), B=D19
Encoder M2: A=D20 (INT3), B=D21
Encoder M3: A=A0,         B=A1
Encoder M4: A=A2,         B=A3
```

#### PCA9685 Servo Driver (Robotic Arm)
```
I2C Communication:
  SDA → A4
  SCL → A5
  VCC → 5V
  GND → GND

Servo Channels:
  Channel 0 → Joint 1 (base rotation)
  Channel 1 → Joint 2 (shoulder)
  Channel 2 → Joint 3 (elbow)
  Channel 4 → Joint 4 (wrist)
  Channel 5 → Gripper base
  Channel 6 → Gripper gear
```

#### Power Supply
```
Arduino:      USB power (5V)
L298N:        12V external (motors)
PCA9685:      5V external (servos - separate from Arduino!)
Servos:       Connect to PCA9685 servo outputs
```

**⚠️ IMPORTANT**: Never power servos from Arduino 5V pin - use external 5V regulator!

### Uploading Firmware

```bash
# Open Arduino IDE
# File → Open → mobile_manipulator_bringup/arduino/unified_mobile_manipulator.ino
# Tools → Board → Arduino Uno
# Tools → Port → /dev/ttyACM0
# Sketch → Upload
```

### Serial Port Configuration

```bash
# Find Arduino port
ls /dev/ttyACM*

# Check permissions
ls -l /dev/ttyACM0

# Add user to dialout group (if needed)
sudo usermod -a -G dialout $USER
# Log out and back in

# Create udev rule for persistent naming
sudo nano /etc/udev/rules.d/99-mobile-manipulator.rules

# Add:
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0043", SYMLINK+="mobile_manipulator"

# Reload rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Now use /dev/mobile_manipulator instead of /dev/ttyACM0
```

---

## 📡 ROS2 Topics

### Mobile Base Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands (input) |
| `/odom` | nav_msgs/Odometry | Odometry with covariance |
| `/scan` | sensor_msgs/LaserScan | Lidar data |
| `/camera/image_raw` | sensor_msgs/Image | Camera RGB image |

### Robotic Arm Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/arm/joint_commands` | sensor_msgs/JointState | Direct joint commands |
| `/joint_states` | sensor_msgs/JointState | Current joint positions |
| `/arm_controller/follow_joint_trajectory` | action | MoveIt trajectory execution |

### System Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/hardware/status` | std_msgs/String | System status messages |
| `/diagnostics` | diagnostic_msgs/DiagnosticArray | Hardware diagnostics |

---

## 🗺️ TF Tree

```
map (from SLAM/localization)
 └─ odom (from encoder odometry)
     └─ body_link (robot base frame)
         ├─ lidar_link (laser scanner)
         ├─ camera_link (RGB camera)
         ├─ front_right_joint (wheel)
         ├─ front_left_joint (wheel)
         ├─ back_right_joint (wheel)
         ├─ back_left_joint (wheel)
         └─ arm_base_link (arm mount point)
             └─ link_1 (joint_1)
                 └─ link_2 (joint_2)
                     └─ link_3 (joint_3)
                         └─ link_4 (joint_4)
                             └─ gripper_base_link
                                 ├─ left_gear_link
                                 └─ right_gear_link
```

---

## ⚙️ Configuration Parameters

### Hardware Bridge Parameters

Edit: `config/hardware/hardware_params.yaml`

```yaml
unified_hardware_bridge:
  ros__parameters:
    serial_port: /dev/mobile_manipulator
    baud_rate: 115200
    encoder_ticks_per_rev: 360        # Adjust for your encoders
    wheel_radius: 0.075               # meters
    wheel_base: 0.67                  # meters (left to right)
    publish_rate: 50.0                # Hz
    cmd_timeout: 0.5                  # seconds
```

### MoveIt Parameters

Edit: `config/arm/moveit_config.yaml`

- Planning groups: `arm`, `gripper`
- Kinematics solver: KDL
- Planning pipeline: OMPL
- Controllers: `arm_controller`, `gripper_controller`

### Nav2 Parameters

Edit: `config/base/nav2_params.yaml`

- Costmap configuration
- Planner: NavFn or SMAC
- Controller: DWB
- Recovery behaviors

---

## 🧪 Testing

### Test Arduino Communication

```bash
# Serial monitor test
ros2 run mobile_manipulator_bringup test_arduino.py

# Expected output:
# STATUS,READY,Unified Mobile Manipulator Controller
# HEARTBEAT,1000
# ...
```

### Test Base Motors

```bash
# Launch hardware bridge
ros2 launch mobile_manipulator_bringup hardware/base_hardware.launch.py

# Send velocity command
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"

# Check odometry
ros2 topic echo /odom
```

### Test Arm Servos

```bash
# Launch arm hardware
ros2 launch mobile_manipulator_bringup hardware/arm_hardware.launch.py

# Send joint command
ros2 topic pub /arm/joint_commands sensor_msgs/JointState "{name: ['joint_1'], position: [1.57]}"

# Check joint states
ros2 topic echo /joint_states
```

### Test MoveIt Integration

```bash
# Launch MoveIt with hardware
ros2 launch mobile_manipulator_bringup bringup.launch.py mode:=arm_only

# In MoveIt RViz:
# - Select "Planning" tab
# - Set goal state or use interactive markers
# - Plan and Execute
```

---

## 🐛 Troubleshooting

### Issue: Arduino not detected

**Symptoms**: `/dev/ttyACM0` not found

**Solutions**:
1. Check USB connection
2. Verify Arduino is powered
3. Check permissions: `sudo chmod 666 /dev/ttyACM0`
4. Add user to dialout group: `sudo usermod -a -G dialout $USER`

### Issue: Motors not moving

**Symptoms**: Commands sent but no motor response

**Solutions**:
1. Check L298N power supply (12V connected?)
2. Verify enable pins are HIGH
3. Check MIN_PWM threshold in Arduino code
4. Test motors directly with Arduino Serial Monitor

### Issue: Servos jittering

**Symptoms**: Servos vibrate or move erratically

**Solutions**:
1. Check PCA9685 power supply (stable 5V?)
2. Verify I2C connections (SDA/SCL)
3. Reduce servo movement speed in firmware
4. Check for electrical interference

### Issue: Odometry drifting

**Symptoms**: Robot position estimate diverges from reality

**Solutions**:
1. Calibrate encoder ticks per revolution
2. Check wheel radius and wheel base parameters
3. Verify encoders are properly attached to wheels
4. Use AMCL localization with map for correction

### Issue: MoveIt planning fails

**Symptoms**: "No valid plan found"

**Solutions**:
1. Check joint limits in URDF
2. Verify collision geometry
3. Increase planning timeout
4. Try different planner (RRTConnect, EST, etc.)
5. Check if goal is reachable (IK solution exists)

---

## 📚 Related Packages

This package integrates with:

- **four_wheel_bot**: Base robot description and navigation
- **mobile_arm_manipulator_config**: Arm MoveIt configuration
- **ros2_arduino_bridge**: Legacy Arduino bridges (now unified)

---

## 🤝 Contributing

Contributions welcome! Areas for improvement:

- [ ] Add IMU integration for better odometry
- [ ] Implement force/torque sensing on arm
- [ ] Add depth camera support
- [ ] Create autonomous manipulation demos
- [ ] Improve calibration procedures

---

## 📄 License

BSD 3-Clause License

---

## 👥 Authors

- Mobile Manipulator Integration Team
- Based on four_wheel_bot and mobile_arm_manipulator_config

---

## 📞 Support

For issues and questions:
- Check this README first
- Review Arduino serial output for errors
- Test components individually before integration
- Check ROS2 logs: `ros2 log`

**Hardware debugging**:
- Use Arduino Serial Monitor for direct testing
- Multimeter for voltage verification
- Oscilloscope for PWM signal verification (advanced)

---

## 🔄 Update Log

**Version 1.0.0** (2025-01-12)
- Initial unified system integration
- Single Arduino firmware for base + arm
- Encoder-based odometry
- Full MoveIt and Nav2 integration
- Modular URDF structure

---

**END OF README**
