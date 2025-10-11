cmake_minimum_required(VERSION 3.8)
project(cex_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)  # Add rclpy as a dependency
find_package(rosidl_default_generators REQUIRED)  # for generating messages and services 
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED) #action msg is different from std msg

set(action_files "action/MoveTurtle.action")


#Generate service files from server
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NumPair.msg"     #a custom msg type
  "srv/AddTwoInt.srv"   #server data type
  ${action_files}
  DEPENDENCIES builtin_interfaces action_msgs
  DEPENDENCIES std_msgs
)


#install all python scripts inside script
file(GLOB PYTHON_SCRIPTS "script/*.py")

install(
  PROGRAMS
  ${PYTHON_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

#Install the server directory
install(
  DIRECTORY srv 
  DESTINATION share/${PROJECT_NAME}
)

#install launch file
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}

)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
