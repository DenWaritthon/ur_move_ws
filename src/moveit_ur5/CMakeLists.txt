cmake_minimum_required(VERSION 3.5)
project(moveit_ur5)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################ FIND DEPENDENCIES  #################
find_package(ament_cmake REQUIRED)		# cpp (optional)
find_package(rclcpp REQUIRED)			# cpp (optional)
find_package(ament_cmake_python REQUIRED)   	# python (optional)
find_package(rclpy REQUIRED)                	# python (optional)
find_package(moveit_ros_planning_interface REQUIRED)  # cpp (required)
# find_package(moveit_visual_tools REQUIRED) 	# cpp (optional)
######################################################

# Install Python Module
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/dummy_script.py
  DESTINATION lib/${PROJECT_NAME}
)


# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(hello_moveit src/hello_moveit.cpp)
# ament_target_dependencies(hello_moveit moveit_ros_planning_interface moveit_visual_tools rclcpp)
ament_target_dependencies(
  hello_moveit
  "moveit_ros_planning_interface"
  # "moveit_visual_tools"
  "rclcpp"
)

# Install Cpp executables
install(TARGETS
  hello_moveit
  DESTINATION lib/${PROJECT_NAME}
)

################ INSTALL LAUNCH, ETC #################
install(DIRECTORY
  # add directories here
  DESTINATION share/${PROJECT_NAME})

ament_package()
