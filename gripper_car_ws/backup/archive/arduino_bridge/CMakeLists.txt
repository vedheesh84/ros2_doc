cmake_minimum_required(VERSION 3.8)
project(arduino_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/hardware_bridge.py
  scripts/arm_servo_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install Arduino firmware (as documentation/reference)
install(DIRECTORY arduino/
  DESTINATION share/${PROJECT_NAME}/arduino
)

ament_package()
