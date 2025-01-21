cmake_minimum_required(VERSION 3.8)
project(unm_ros2_vention)

# -- 1) Find the required CMake/ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# (Optional) If your code uses std_msgs, sensor_msgs, etc., add them:
# find_package(std_msgs REQUIRED)

# -- 2) Install Python modules in this package
ament_python_install_package(${PROJECT_NAME})

# -- 3) Declare Python executables for each node you want to run via "ros2 run"
# vention_node   -> the main node
# vention_reset  -> the optional ephemeral reset node

install(
  PROGRAMS
    scripts/vention_node.py
    scripts/vention_reset.py
    scripts/vention_api_wrapper.py
    scripts/MachineMotion.py
  DESTINATION lib/${PROJECT_NAME}
)


# -- 4) Install launch files so you can "ros2 launch unm_ros2_vention file.launch.py"
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# -- 5) (Optional) Install config directory if you have param YAML or other config
#install(
#  DIRECTORY config
#  DESTINATION share/${PROJECT_NAME}
#)

# -- 6) Finalize package
ament_package()
