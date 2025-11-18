cmake_minimum_required(VERSION 3.22)
project(mobile_arm_manipulator_config)

find_package(ament_cmake REQUIRED)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/urdf")
  install(
    DIRECTORY urdf
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.urdf" EXCLUDE)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/models")
  install(
    DIRECTORY models
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.models" EXCLUDE)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/meshes")
  install(
    DIRECTORY meshes
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.meshes" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})


# Install Python scripts
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts")
  install(
    PROGRAMS scripts/arduino_ros2_bridge.py
    scripts/ar_test.py
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# Install Arduino sketch
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/arduino")
  install(
    DIRECTORY arduino
    DESTINATION share/${PROJECT_NAME}
  )
endif()
