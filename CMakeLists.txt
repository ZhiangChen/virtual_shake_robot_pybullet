cmake_minimum_required(VERSION 3.8)
project(virtual_shake_robot_pybullet)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "action/AF.action"
  "action/TrajectoryAction.action"
  "action/LoadPBR.action"
  "srv/ManageModel.srv"
  "action/RecordingAction.action"
  "action/LoadDispl.action"
  DEPENDENCIES builtin_interfaces std_msgs
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Install Python modules
install(DIRECTORY
    src/virtual_shake_robot_pybullet/
    DESTINATION lib/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.py"
)



install(PROGRAMS
  src/simulation_node.py
  src/control_node.py
  src/pulse_motion_client.py
  src/perception_node.py
  src/pbr_loader.py
  src/data_loader.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()