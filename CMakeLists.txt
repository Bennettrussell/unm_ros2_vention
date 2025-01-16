cmake_minimum_required(VERSION 3.5)
project(vention_control)

# 1. Load macros and functions from ament
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# 2. ROS 2 Python library
find_package(rclpy REQUIRED)

# 3. Messages & Services dependencies
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 4. Generate interfaces for custom msg/srv
#    This replaces "message_generation" from ROS 1
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointCommand.msg"
  "srv/PositionMove.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

# 5. Mark the Python package for installation
#    This uses standard Python packaging; ensures your Python module is installed.
ament_python_install_package(${PROJECT_NAME})

# 6. Install Python scripts as executables
#    These will be made runnable via "ros2 run vention_control <script_name>"
install(
  PROGRAMS
    scripts/vention_node.py
    scripts/zero_vention_carriages.py
    scripts/draw_sine.py
    scripts/draw_sine_2.py
  DESTINATION lib/${PROJECT_NAME}
)

# 7. Export the generated interfaces and finalize
ament_export_dependencies(rosidl_default_runtime)
ament_package()
