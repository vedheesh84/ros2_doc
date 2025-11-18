# Mobile Manipulator - QUICKSTART Guide

## 🚀 Quick Test of Current Implementation

This guide helps you test the completed components **right now**, even though the full implementation is still in progress.

---

## ✅ What's Ready to Test

1. **Arduino Firmware** - Upload and test motor/servo control
2. **ROS2 Hardware Bridge** - Test communication with Arduino
3. **URDF Visualization** - View robot model in RViz

---

## 📋 Prerequisites

```bash
# Install dependencies
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-xacro
sudo apt install python3-serial

# Build the package
cd ~/ros2_doc/gripper_car_ws
colcon build --packages-select mobile_manipulator_bringup
source install/setup.bash
```

---

## 🔌 Test 1: Arduino Firmware

### Upload Firmware

1. Open Arduino IDE
2. Install library: **Adafruit PWM Servo Driver Library**
3. Open: `mobile_manipulator_bringup/arduino/unified_mobile_manipulator.ino`
4. Select Board: **Arduino Uno**
5. Select Port: `/dev/ttyACM0`
6. Click **Upload**

### Test Serial Communication

```bash
# Open serial monitor (115200 baud)
# You should see:
# STATUS,READY,Unified Mobile Manipulator Controller
# HEARTBEAT,1000
# HEARTBEAT,2000
# ...

# Send test commands:
VEL,0.2,0.0          # Move forward
STOP                 # Stop
SERVO,0,90           # Move joint_1 to center
GET_STATUS           # Get system status
```

---

## 🤖 Test 2: ROS2 Hardware Bridge

### Launch Bridge (without full launch files)

```bash
# Terminal 1: Run bridge directly
ros2 run mobile_manipulator_bringup unified_hardware_bridge.py \
  --ros-args \
  -p serial_port:=/dev/ttyACM0 \
  -p baud_rate:=115200

# You should see:
# [INFO] [unified_hardware_bridge]: Unified Hardware Bridge initialized
# [INFO] [unified_hardware_bridge]: ✓ Connected to Arduino
```

### Test Topics

```bash
# Terminal 2: List topics
ros2 topic list
# Expected:
# /cmd_vel
# /odom
# /joint_states
# /arm/joint_commands
# /hardware/status

# Terminal 3: Send velocity command
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" --once

# Terminal 4: Echo odometry
ros2 topic echo /odom --once

# Terminal 5: Echo joint states
ros2 topic echo /joint_states --once

# Terminal 6: Send arm command
ros2 topic pub /arm/joint_commands sensor_msgs/JointState \
  "{name: ['joint_1'], position: [1.57]}" --once
```

---

## 👁️ Test 3: URDF Visualization

### Visualize Robot Model

```bash
# Generate URDF from xacro
ros2 run xacro xacro \
  src/mobile_manipulator_bringup/urdf/mobile_manipulator.urdf.xacro \
  use_sim:=false \
  > /tmp/mobile_manipulator.urdf

# Check for errors
check_urdf /tmp/mobile_manipulator.urdf

# Launch RViz with joint state publisher
ros2 launch joint_state_publisher_gui joint_state_publisher_gui.launch.py \
  robot_description:=/tmp/mobile_manipulator.urdf
```

**Alternative: Manual visualization**

```bash
# Terminal 1: Publish robot description
ros2 param set /robot_state_publisher robot_description \
  "$(xacro src/mobile_manipulator_bringup/urdf/mobile_manipulator.urdf.xacro use_sim:=false)"

# Terminal 2: Launch robot state publisher
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(xacro src/mobile_manipulator_bringup/urdf/mobile_manipulator.urdf.xacro)"

# Terminal 3: Launch joint state publisher GUI
ros2 run joint_state_publisher_gui joint_state_publisher_gui

# Terminal 4: Launch RViz
rviz2
# In RViz:
# - Add -> RobotModel
# - Fixed Frame: body_link
# - Move sliders in joint_state_publisher_gui
```

---

## 🛠️ Troubleshooting

### Issue: "Permission denied" on /dev/ttyACM0

```bash
sudo chmod 666 /dev/ttyACM0
# Or permanently:
sudo usermod -a -G dialout $USER
# Then log out and back in
```

### Issue: "Package not found"

```bash
cd ~/ros2_doc/gripper_car_ws
colcon build --packages-select mobile_manipulator_bringup
source install/setup.bash
```

### Issue: Python module not found

```bash
pip3 install pyserial numpy
```

### Issue: Arduino not responding

1. Check USB connection
2. Verify correct port: `ls /dev/ttyACM*`
3. Check baud rate: 115200
4. Re-upload firmware
5. Press Arduino reset button

### Issue: URDF xacro errors

```bash
# Check xacro syntax
ros2 run xacro xacro --check-order \
  src/mobile_manipulator_bringup/urdf/mobile_manipulator.urdf.xacro
```

---

## 📊 Expected Behavior

### Hardware Bridge Output

```
[INFO] [unified_hardware_bridge]: Unified Hardware Bridge initialized
[INFO] [unified_hardware_bridge]: Serial port: /dev/ttyACM0 @ 115200 baud
[INFO] [unified_hardware_bridge]: Wheel radius: 0.075m, base: 0.67m
[INFO] [unified_hardware_bridge]: ✓ Connected to Arduino
[INFO] [unified_hardware_bridge]: Arduino: STATUS,READY,Unified Mobile Manipulator Controller
```

### Arduino Serial Output

```
STATUS,READY,Unified Mobile Manipulator Controller
INFO,Base: 4-wheel differential drive with encoders
INFO,Arm: 6-DOF with PCA9685 servo driver
HEARTBEAT,1000
ODOM,0,0,0,0,1234
HEARTBEAT,2000
ODOM,12,15,8,10,2345
...
```

### ROS2 Topics

```bash
$ ros2 topic hz /odom
average rate: 20.032
$ ros2 topic hz /joint_states
average rate: 50.015
```

---

## 🎯 Next Steps After Testing

Once you've verified the above works:

1. **Configure MoveIt** - Set up motion planning
2. **Add Nav2** - Enable autonomous navigation
3. **Create Launch Files** - Automated startup
4. **Calibrate** - Tune parameters for your hardware
5. **Integrate** - Combine all subsystems

---

## 📞 Need Help?

**Check:**
- README.md - Full documentation
- IMPLEMENTATION_STATUS.md - What's done and what's pending
- Arduino serial output for error messages
- ROS2 logs: `ros2 log`

**Common Issues:**
- Encoders not connected → Odometry will be 0
- Servos not moving → Check PCA9685 power supply
- Motors not responding → Check L298N connections and power

---

**Ready to continue with full implementation?**
The remaining tasks are in IMPLEMENTATION_STATUS.md

