cmake_minimum_required(VERSION 3.8)
project(iii_drone_core)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
     set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

set(III_DRONE_INCLUDE_DIRS
  "${PROJECT_SOURCE_DIR}/include/"
)

include_directories(src/matlab_MPC_5Hz_hp10)
link_directories(src/matlab_MPC_5Hz_hp10)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(iii_drone_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(nav_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_components
  Eigen3
  sensor_msgs
  px4_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  OpenCV
  cv_bridge
  image_transport
  iii_drone_interfaces
)

# Library:
set(III_DRONE_LIBRARY_HEADERS
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/configuration/configurator.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/configuration/tf/tf_configurator.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/configuration/mpc/mpc_configurator.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/perception/hough_transformer_node/hough_transformer_node_configurator.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/perception/pl_dir_computer_node/pl_dir_computer_node_configurator.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/perception/pl_mapper_node/pl_mapper_node_configurator.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/trajectory_controller_node/trajectory_controller_node_configurator.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/utils/math.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/utils/blocking_queue.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/perception/single_line.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/perception/powerline.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/mpc_parameters.hpp"
)

set(III_DRONE_LIBRARY_SOURCES
  "${PROJECT_SOURCE_DIR}/src/configuration/configurator.cpp"
  "${PROJECT_SOURCE_DIR}/src/configuration/tf/tf_configurator.cpp"
  "${PROJECT_SOURCE_DIR}/src/configuration/mpc/mpc_configurator.cpp"

  "${PROJECT_SOURCE_DIR}/src/perception/hough_transformer_node/hough_transformer_node_configurator.cpp"
  "${PROJECT_SOURCE_DIR}/src/perception/pl_dir_computer_node/pl_dir_computer_node_configurator.cpp"
  "${PROJECT_SOURCE_DIR}/src/perception/pl_mapper_node/pl_mapper_node_configurator.cpp"

  "${PROJECT_SOURCE_DIR}/src/control/trajectory_controller_node/trajectory_controller_node_configurator.cpp"

  "${PROJECT_SOURCE_DIR}/src/utils/math.cpp"

  "${PROJECT_SOURCE_DIR}/src/perception/single_line.cpp"
  "${PROJECT_SOURCE_DIR}/src/perception/powerline.cpp"

  "${PROJECT_SOURCE_DIR}/src/control/mpc_parameters.cpp"
)

include_directories(${III_DRONE_INCLUDE_DIRS})

add_library(iii_drone_lib 
  SHARED
  ${III_DRONE_LIBRARY_HEADERS}
  ${III_DRONE_LIBRARY_SOURCES}
)

target_sources(iii_drone_lib
  PUBLIC
  ${III_DRONE_LIBRARY_HEADERS}
  ${III_DRONE_LIBRARY_SOURCES}
)

target_include_directories(iii_drone_lib
  PUBLIC
  ${III_DRONE_INCLUDE_DIRS}
)

ament_target_dependencies(iii_drone_lib
  ${dependencies}
)

ament_export_libraries(iii_drone_lib)
ament_export_include_directories(${III_DRONE_INCLUDE_DIRS})

install(TARGETS iii_drone_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY "${PROJECT_SOURCE_DIR}/include/"
  DESTINATION include
)

# MATLAB MPC Library:
add_subdirectory("${PROJECT_SOURCE_DIR}/src/extern")

# Executables:
function(custom_cpp_target target_name target_sources)
  # Loop over optional arguments and add to target sources:
  foreach(arg ${ARGN})
    list(APPEND target_sources "${arg}")
  endforeach()

  # Print target sources:
  message(STATUS "Target sources for ${target_name}:")
  foreach(source ${target_sources})
    message(STATUS "  ${source}")
  endforeach()

  add_executable(${target_name}
    "${target_sources}"
  )

  add_dependencies(${target_name}
    iii_drone_lib
  )

  target_include_directories(${target_name}
    PUBLIC
    "${III_DRONE_INCLUDE_DIRS}"
  )

  target_link_libraries(${target_name}
    iii_drone_lib
  )

  ament_target_dependencies(${target_name}
    ${dependencies}
  )

  target_link_libraries(${target_name} "${cpp_typesupport_target}")

  install(TARGETS ${target_name}
    DESTINATION lib/${PROJECT_NAME}
  )

endfunction()

# Drone frame broadcaster:
custom_cpp_target(drone_frame_broadcaster
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/utils/drone_frame_broadcaster_node/drone_frame_broadcaster_node.hpp"
  "${PROJECT_SOURCE_DIR}/src/utils/drone_frame_broadcaster_node/drone_frame_broadcaster_node.cpp"
)

# Hough transformer:
custom_cpp_target(hough_transformer
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/perception/hough_transformer_node/hough_transformer_node.hpp"
  "${PROJECT_SOURCE_DIR}/src/perception/hough_transformer_node/hough_transformer_node.cpp"
)

# PL dir computer:
custom_cpp_target(pl_dir_computer
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/perception/pl_dir_computer_node/pl_dir_computer_node.hpp"
  "${PROJECT_SOURCE_DIR}/src/perception/pl_dir_computer_node/pl_dir_computer_node.cpp"
)

# PL mapper:
custom_cpp_target(pl_mapper
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/perception/pl_mapper_node/pl_mapper_node.hpp"
  "${PROJECT_SOURCE_DIR}/src/perception/pl_mapper_node/pl_mapper_node.cpp"
)

# Trajectory controller:
custom_cpp_target(trajectory_controller 
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/trajectory_controller_node/trajectory_controller_node.hpp"
  "${PROJECT_SOURCE_DIR}/src/control/trajectory_controller_node/trajectory_controller_node.cpp"
)
target_include_directories(trajectory_controller 
  PRIVATE
  ${MATLAB_MPC_INCLUDE_DIRS}
)
target_link_libraries(trajectory_controller 
  matlab_mpc
)

# # Continuous mission orchestrator:
# custom_cpp_target(continuous_mission_orchestrator
#   "${PROJECT_SOURCE_DIR}/include/iii_drone_core/mission/continuous_mission_orchestrator_node/continuous_mission_orchestrator_node.hpp"
#   "${PROJECT_SOURCE_DIR}/src/mission/continuous_mission_orchestrator_node/continuous_mission_orchestrator_node.cpp"
# )

############
# Install ##
############
# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  "${PROJECT_SOURCE_DIR}/scripts/payload/charger_gripper_node/charger_gripper_node.py"
  "${PROJECT_SOURCE_DIR}/scripts/configuration/configuration_server_node/configuration_server_node.py"
  DESTINATION lib/${PROJECT_NAME}
)

# Export information to downstream packages
if(ROS_DISTRO IN_LIST ROS_DISTROS)
  ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime eigen3_cmake_module Eigen3 px4_msgs geometry_msgs sensor_msgs OpenCV)
else()
  ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime Eigen3 px4_msgs geometry_msgs sensor_msgs)
endif()

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# install the config directory
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
