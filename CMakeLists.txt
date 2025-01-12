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
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointCommand.msg"
  "srv/PositionMove.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

# 5. Mark the Python package for installation
ament_python_install_package(${PROJECT_NAME})

# 6. Install Python scripts as executables
install(
  PROGRAMS
    scripts/vention_node.py
    scripts/zero_vention_carriages.py
    scripts/draw_sine.py
    scripts/draw_sine_2.py
  DESTINATION lib/${PROJECT_NAME}
)

# 7. Export message runtime and finalize
ament_export_dependencies(rosidl_default_runtime)
ament_package()
