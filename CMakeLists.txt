cmake_minimum_required(VERSION 3.8)
project(unm_ros2_vention)

# -- 1) Find the required CMake/ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# (Optional) If your code uses std_msgs, sensor_msgs, etc., add them:
# find_package(std_msgs REQUIRED)

# -- 2) Install Python modules in this package
ament_python_install_package(${PROJECT_NAME})

# -- 3) Declare Python executables for each node you want to run via "ros2 run"
# vention_node   -> the main node
# vention_reset  -> the optional ephemeral reset node
ament_add_python_executable(vention_node src/vention_node.py)
ament_add_python_executable(vention_reset src/vention_reset.py)


# -- 4) Specify where to install these executables
install(
  TARGETS
    vention_node
    vention_reset
  DESTINATION lib/${PROJECT_NAME}
)

# -- 5) Install launch files (if any) so you can "ros2 launch unm_ros2_vention file.launch.py"
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# -- 6) (Optional) Install config directory if you have param YAML or other config
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# -- 7) Finalize package
ament_package()
