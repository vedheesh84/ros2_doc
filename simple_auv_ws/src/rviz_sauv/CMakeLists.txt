cmake_minimum_required(VERSION 3.8)
project(rviz_sauv)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files (if directory exists)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# Install URDF files (if directory exists)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/urdf")
  install(
    DIRECTORY urdf
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# Install config files (if directory exists)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# Install mesh files (if directory exists)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/meshes")
  install(
    DIRECTORY meshes
    DESTINATION share/${PROJECT_NAME}
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
