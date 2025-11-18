# Mobile Manipulator - Quick Start Guide

## 🚀 5-Minute Setup

### 1. Upload Arduino Code

```bash
# Open in Arduino IDE:
mobile_manipulator_bringup/arduino/unified_mobile_manipulator_mega.ino

# Select: Board → Arduino Mega 2560
# Select: Port → /dev/ttyACM0
# Click: Upload
```

### 2. Build ROS2 Package

```bash
cd ~/ros2_doc/gripper_car_ws
colcon build --packages-select mobile_manipulator_bringup
source install/setup.bash
```

### 3. Launch System

```bash
ros2 launch mobile_manipulator_bringup complete_system.launch.py
```

**Done!** 🎉

---

## 🎮 Control the Robot

### Keyboard Controls

The integrated teleop opens automatically. Press **'m'** to toggle modes:

**BASE MODE** (drive the robot):
- `i/,` - forward/backward
- `j/l` - left/right
- `k` - stop

**ARM MODE** (control the arm):
- `1-6` - select joint
- `+/-` - move joint
- `h` - home position
- `g/f` - open/close gripper

**SPACE** - Emergency stop!

---

## 📋 Files Created

### Arduino Code
```
mobile_manipulator_bringup/arduino/unified_mobile_manipulator_mega.ino
```
**Combined four-wheel car + 6-DOF arm controller**
- L298N motor drivers (4 motors)
- PCA9685 servo driver (6 servos)
- Quadrature encoders (4 wheels)
- Serial communication @ 115200 baud

### ROS2 Python Scripts

1. **unified_hardware_bridge.py** (already exists)
   - Arduino ↔ ROS2 communication
   - Publishes: `/odom`, `/joint_states`
   - Subscribes: `/cmd_vel`, `/arm/joint_commands`

2. **integrated_teleop_control.py** (NEW)
   - Keyboard control for base and arm
   - Toggle between modes with 'm'
   - Emergency stop with SPACE

3. **moveit_arm_controller.py** (NEW)
   - MoveIt2 integration
   - Predefined poses (HOME, READY, PICK, PLACE)
   - Gripper control
   - Python API for custom programs

### Launch Files

1. **complete_system.launch.py** (NEW)
   - Master launch file - starts everything
   - Arguments: `serial_port`, `use_rviz`, `use_moveit`, etc.

2. **hardware_bringup.launch.py** (NEW)
   - Hardware bridge + robot description
   - No teleop or MoveIt

3. **teleop_control.launch.py** (NEW)
   - Just keyboard control

4. **moveit_control.launch.py** (NEW)
   - Just MoveIt arm controller

---

## 🔧 Common Commands

### Start Full System
```bash
ros2 launch mobile_manipulator_bringup complete_system.launch.py
```

### Start Hardware Only
```bash
ros2 launch mobile_manipulator_bringup hardware_bringup.launch.py \
  serial_port:=/dev/ttyACM0
```

### Test Individual Components

```bash
# Hardware bridge only
ros2 run mobile_manipulator_bringup unified_hardware_bridge.py

# Teleop only
ros2 run mobile_manipulator_bringup integrated_teleop_control.py

# MoveIt controller only
ros2 run mobile_manipulator_bringup moveit_arm_controller.py
```

### Monitor Topics

```bash
# List all topics
ros2 topic list

# Watch odometry
ros2 topic echo /odom

# Watch joint states
ros2 topic echo /joint_states

# Send velocity command
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.2, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}"
```

---

## 🔌 Hardware Connections Quick Reference

### Motor Drivers (L298N)
```
Driver 1 (Back Motors):
  BL: IN1=Pin2, IN2=Pin3, EN=Pin10
  BR: IN1=Pin4, IN2=Pin5, EN=Pin11

Driver 2 (Front Motors):
  FL: IN1=Pin6, IN2=Pin7, EN=Pin12
  FR: IN1=Pin8, IN2=Pin9, EN=Pin13
```

### Encoders
```
BL: A=Pin18, B=Pin19
BR: A=Pin20, B=Pin21
FL: A=Pin2,  B=Pin22  ⚠️ Pin conflict!
FR: A=Pin3,  B=Pin23  ⚠️ Pin conflict!
```

**⚠️ WARNING:** Pins 2 & 3 are shared! Move motor pins or encoder pins to avoid conflicts.

### PCA9685 (I2C)
```
SDA → A4
SCL → A5

Servos:
  Ch0 → joint_1 (base)
  Ch1 → joint_2 (shoulder)
  Ch2 → joint_3 (elbow)
  Ch4 → joint_4 (wrist)
  Ch5 → gripper_base
  Ch6 → gripper_gear
```

---

## 🐛 Quick Troubleshooting

### Arduino won't connect
```bash
sudo chmod 666 /dev/ttyACM0
sudo usermod -a -G dialout $USER
```

### Motors don't move
- Check 12V power to L298N
- Verify motor connections
- Test with: `VEL,0.2,0.0` in Serial Monitor

### Servos don't move
- Check 5V servo power (separate from Arduino!)
- Verify I2C connections (A4/A5)
- Test with: `SERVO,0,90` in Serial Monitor

### Odometry drifts
- Check encoder wiring
- Calibrate wheel_radius parameter
- Monitor encoder counts

### ROS2 topics missing
- Check node is running: `ros2 node list`
- Restart bridge with debug: `--log-level debug`
- Verify serial port: `ls -l /dev/ttyACM*`

---

## 📚 Documentation

- **Full Guide:** `INTEGRATION_GUIDE.md`
- **Arduino Code:** `arduino/unified_mobile_manipulator_mega.ino`
- **Existing Docs:** `mobile_arm_manipulator_config/ARDUINO_INTEGRATION.md`

---

## 🎯 Next Steps

1. **Calibrate the system:**
   - Measure actual wheel movement vs commanded
   - Adjust `wheel_radius` and `wheel_base`
   - Test servo range and adjust pulse widths

2. **Create custom programs:**
   - Use `moveit_arm_controller.py` as template
   - Define custom poses in `ArmPose` enum
   - Write pick-and-place sequences

3. **Add navigation:**
   - Enable Nav2: `use_nav:=true`
   - Create map with SLAM
   - Set navigation goals

4. **Integrate sensors:**
   - Add camera/lidar processing
   - Implement object detection
   - Add force/torque sensing

---

## 🆘 Support

If you encounter issues:

1. Check `INTEGRATION_GUIDE.md` for detailed troubleshooting
2. Verify all hardware connections
3. Test components individually before integrating
4. Check ROS2 logs: `ros2 run rqt_console rqt_console`

---

**Happy Building! 🤖**
