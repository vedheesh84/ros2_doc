cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ============================================================================
# LAUNCH FILES
# ============================================================================

# Install hardware launch files
install(DIRECTORY launch/hardware/
  DESTINATION share/${PROJECT_NAME}/launch/hardware
  FILES_MATCHING PATTERN "*.launch.py"
)

# Install simulation launch files
install(DIRECTORY launch/sim/
  DESTINATION share/${PROJECT_NAME}/launch/sim
  FILES_MATCHING PATTERN "*.launch.py"
)


# Install gazebo launch files
install(DIRECTORY launch/gazebo/
  DESTINATION share/${PROJECT_NAME}/launch/gazebo
  FILES_MATCHING PATTERN "*.launch.py"
)


# Install master launch files
install(PROGRAMS
  launch/bringup.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

# ============================================================================
# CONFIGURATION FILES
# ============================================================================

# Install arm configuration files
install(DIRECTORY config/arm/
  DESTINATION share/${PROJECT_NAME}/config/arm
  FILES_MATCHING
  PATTERN "*.yaml"
  PATTERN "*.srdf"
)

# Install hardware configuration files
install(DIRECTORY config/hardware/
  DESTINATION share/${PROJECT_NAME}/config/hardware
  FILES_MATCHING PATTERN "*.yaml"
)

# Install MoveIt configuration files
install(DIRECTORY config/moveit/
  DESTINATION share/${PROJECT_NAME}/config/moveit
  FILES_MATCHING
  PATTERN "*.yaml"
  PATTERN "*.rviz"
)

# Install Gazebo configuration files (if exists)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config/gazebo)
  install(DIRECTORY config/gazebo/
    DESTINATION share/${PROJECT_NAME}/config/gazebo
    FILES_MATCHING
    PATTERN "*.yaml"
    PATTERN "*.rviz"
  )
endif()

# ============================================================================
# ROBOT DESCRIPTION FILES
# ============================================================================

# Install URDF/xacro files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
  FILES_MATCHING
  PATTERN "*.urdf"
  PATTERN "*.xacro"
  PATTERN "*.csv"
)

# Install mesh files
install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
  FILES_MATCHING
  PATTERN "*.STL"
  PATTERN "*.stl"
  PATTERN "*.dae"
  PATTERN "*.obj"
)

# ============================================================================
# SIMULATION FILES
# ============================================================================

# Install world files
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
  FILES_MATCHING PATTERN "*.world"
)

# Install Gazebo models (if exists)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/models)
  install(DIRECTORY models/
    DESTINATION share/${PROJECT_NAME}/models
  )
endif()

# ============================================================================
# SCRIPTS AND EXECUTABLES
# ============================================================================

# Install Python scripts
install(PROGRAMS
  scripts/unified_hardware_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Arduino firmware (as documentation)
install(DIRECTORY arduino/
  DESTINATION share/${PROJECT_NAME}/arduino
  FILES_MATCHING PATTERN "*.ino"
)

# ============================================================================
# DOCUMENTATION
# ============================================================================

# Install documentation files
install(FILES
  README.md
  QUICKSTART.md
  IMPLEMENTATION_STATUS.md
  FINAL_IMPLEMENTATION_SUMMARY.md
  DESTINATION share/${PROJECT_NAME}/
)

# Install .setup_assistant file (if exists)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/.setup_assistant)
  install(FILES .setup_assistant
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()
