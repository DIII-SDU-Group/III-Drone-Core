cmake_minimum_required(VERSION 3.8)
project(iii_drone_core)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
     set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

include(CTest)

# Includes and link:
set(III_DRONE_INCLUDE_DIRS
  "${PROJECT_SOURCE_DIR}/include/"
)

include_directories(src/matlab_MPC_5Hz_hp10)
link_directories(src/matlab_MPC_5Hz_hp10)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclpy REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(iii_drone_interfaces REQUIRED)
find_package(iii_drone_configuration REQUIRED)
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
  rclcpp_lifecycle
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
  iii_drone_configuration
)

# MATLAB MPC Library:
add_subdirectory("${PROJECT_SOURCE_DIR}/src/extern")

# Library files:
set(III_DRONE_LIBRARY_HEADERS
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/utils/math.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/utils/types.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/utils/timestamp.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/utils/atomic.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/utils/history.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/utils/token.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/perception/single_line.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/perception/powerline.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/perception/powerline_direction.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/perception/hough_transformer.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/state.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/reference.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/reference_trajectory.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/kinematic_stop_trajectory.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/cable_aware_trajectory_planner.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/trajectory_generator.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/trajectory_generator_client.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/trajectory_interpolator.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/waypoint_path_planner.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/combined_drone_awareness_handler.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/maneuver/maneuver.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/maneuver/maneuver_types.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/maneuver/maneuver_queue.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/maneuver/maneuver_scheduler.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/maneuver/maneuver_reference_safety_guard.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/maneuver/maneuver_reference_startup_policy.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/maneuver/maneuver_reference_stream_guard.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/maneuver/maneuver_server.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/maneuver/hover_maneuver_server.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/maneuver/hover_by_object_maneuver_server.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/maneuver/hover_on_cable_maneuver_server.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/maneuver/fly_to_position_maneuver_server.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/maneuver/follow_waypoint_path_maneuver_server.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/maneuver/cable_aware_fly_to_position_maneuver_server.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/maneuver/terminal_tracking_hold.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/maneuver/fly_to_object_maneuver_server.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/maneuver/cable_landing_maneuver_server.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/maneuver/cable_takeoff_maneuver_server.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/maneuver/reference_callback_token.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/maneuver/maneuver_request_identity.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/maneuver/maneuver_reference_client.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/adapters/single_line_adapter.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/adapters/projection_plane_adapter.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/adapters/powerline_adapter.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/adapters/point_cloud_adapter.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/adapters/state_adapter.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/adapters/reference_adapter.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/adapters/reference_trajectory_adapter.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/adapters/gripper_status_adapter.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/adapters/target_adapter.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/adapters/maneuver_adapter.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/adapters/combined_drone_awareness_adapter.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/adapters/px4/trajectory_setpoint_adapter.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/adapters/px4/vehicle_odometry_adapter.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/adapters/px4/vehicle_global_position_adapter.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/adapters/px4/vehicle_status_adapter.hpp"

)

set(III_DRONE_LIBRARY_SOURCES
  "${PROJECT_SOURCE_DIR}/src/utils/math.cpp"
  "${PROJECT_SOURCE_DIR}/src/utils/types.cpp"
  "${PROJECT_SOURCE_DIR}/src/utils/timestamp.cpp"

  "${PROJECT_SOURCE_DIR}/src/perception/single_line.cpp"
  "${PROJECT_SOURCE_DIR}/src/perception/powerline.cpp"
  "${PROJECT_SOURCE_DIR}/src/perception/powerline_direction.cpp"
  "${PROJECT_SOURCE_DIR}/src/perception/hough_transformer.cpp"

  "${PROJECT_SOURCE_DIR}/src/control/state.cpp"
  "${PROJECT_SOURCE_DIR}/src/control/reference.cpp"
  "${PROJECT_SOURCE_DIR}/src/control/reference_trajectory.cpp"
  "${PROJECT_SOURCE_DIR}/src/control/kinematic_stop_trajectory.cpp"
  "${PROJECT_SOURCE_DIR}/src/control/terminal_position_tracking_controller.cpp"
  "${PROJECT_SOURCE_DIR}/src/control/cable_aware_trajectory_planner.cpp"
  "${PROJECT_SOURCE_DIR}/src/control/trajectory_generator.cpp"
  "${PROJECT_SOURCE_DIR}/src/control/trajectory_generator_client.cpp"
  "${PROJECT_SOURCE_DIR}/src/control/trajectory_interpolator.cpp"
  "${PROJECT_SOURCE_DIR}/src/control/waypoint_path_planner.cpp"

  "${PROJECT_SOURCE_DIR}/src/control/combined_drone_awareness_handler.cpp"

  "${PROJECT_SOURCE_DIR}/src/control/maneuver/maneuver.cpp"
  "${PROJECT_SOURCE_DIR}/src/control/maneuver/maneuver_types.cpp"
  "${PROJECT_SOURCE_DIR}/src/control/maneuver/maneuver_queue.cpp"

  "${PROJECT_SOURCE_DIR}/src/control/maneuver/maneuver_scheduler.cpp"

  "${PROJECT_SOURCE_DIR}/src/control/maneuver/maneuver_reference_safety_guard.cpp"

  "${PROJECT_SOURCE_DIR}/src/control/maneuver/maneuver_server.cpp"
  "${PROJECT_SOURCE_DIR}/src/control/maneuver/hover_maneuver_server.cpp"
  "${PROJECT_SOURCE_DIR}/src/control/maneuver/hover_by_object_maneuver_server.cpp"
  "${PROJECT_SOURCE_DIR}/src/control/maneuver/hover_on_cable_maneuver_server.cpp"
  "${PROJECT_SOURCE_DIR}/src/control/maneuver/fly_to_position_maneuver_server.cpp"
  "${PROJECT_SOURCE_DIR}/src/control/maneuver/follow_waypoint_path_maneuver_server.cpp"
  "${PROJECT_SOURCE_DIR}/src/control/maneuver/cable_aware_fly_to_position_maneuver_server.cpp"
  "${PROJECT_SOURCE_DIR}/src/control/maneuver/terminal_tracking_hold.cpp"
  "${PROJECT_SOURCE_DIR}/src/control/maneuver/object_tracking_session.cpp"
  "${PROJECT_SOURCE_DIR}/src/control/maneuver/fly_to_object_maneuver_server.cpp"
  "${PROJECT_SOURCE_DIR}/src/control/maneuver/cable_landing_maneuver_server.cpp"
  "${PROJECT_SOURCE_DIR}/src/control/maneuver/cable_takeoff_maneuver_server.cpp"

  "${PROJECT_SOURCE_DIR}/src/control/maneuver/maneuver_reference_client.cpp"
  "${PROJECT_SOURCE_DIR}/src/control/maneuver/maneuver_request_identity.cpp"

  "${PROJECT_SOURCE_DIR}/src/adapters/single_line_adapter.cpp"
  "${PROJECT_SOURCE_DIR}/src/adapters/projection_plane_adapter.cpp"
  "${PROJECT_SOURCE_DIR}/src/adapters/powerline_adapter.cpp"
  "${PROJECT_SOURCE_DIR}/src/adapters/point_cloud_adapter.cpp"
  "${PROJECT_SOURCE_DIR}/src/adapters/state_adapter.cpp"
  "${PROJECT_SOURCE_DIR}/src/adapters/reference_adapter.cpp"
  "${PROJECT_SOURCE_DIR}/src/adapters/reference_trajectory_adapter.cpp"
  "${PROJECT_SOURCE_DIR}/src/adapters/gripper_status_adapter.cpp"
  "${PROJECT_SOURCE_DIR}/src/adapters/target_adapter.cpp"
  "${PROJECT_SOURCE_DIR}/src/adapters/maneuver_adapter.cpp"
  "${PROJECT_SOURCE_DIR}/src/adapters/combined_drone_awareness_adapter.cpp"

  "${PROJECT_SOURCE_DIR}/src/adapters/px4/trajectory_setpoint_adapter.cpp"
  "${PROJECT_SOURCE_DIR}/src/adapters/px4/vehicle_odometry_adapter.cpp"
  "${PROJECT_SOURCE_DIR}/src/adapters/px4/vehicle_global_position_adapter.cpp"
  "${PROJECT_SOURCE_DIR}/src/adapters/px4/vehicle_status_adapter.cpp"

)

option(III_DRONE_OPTIMIZE_CONTROL_KERNELS
  "Optimize the bounded trajectory planner and stop kernels in Debug builds" ON)
if(III_DRONE_OPTIMIZE_CONTROL_KERNELS AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  set_property(SOURCE
    "${PROJECT_SOURCE_DIR}/src/control/trajectory_interpolator.cpp"
    "${PROJECT_SOURCE_DIR}/src/control/kinematic_stop_trajectory.cpp"
    APPEND PROPERTY COMPILE_OPTIONS "$<$<CONFIG:Debug>:-O2>"
  )
endif()

include_directories(
  ${III_DRONE_INCLUDE_DIRS}
  ${MATLAB_MPC_INCLUDE_DIRS}  
)

# III-Drone-Lib:
add_library(iii_drone_lib 
  SHARED
  ${III_DRONE_LIBRARY_HEADERS}
  ${III_DRONE_LIBRARY_SOURCES}
)

target_include_directories(iii_drone_lib
  PUBLIC
  ${III_DRONE_INCLUDE_DIRS}
)
target_include_directories(iii_drone_lib
  PRIVATE
  ${MATLAB_MPC_INCLUDE_DIRS}
)

target_link_libraries(iii_drone_lib
  matlab_mpc
  # yaml-cpp::yaml-cpp
)

ament_target_dependencies(iii_drone_lib
  ${dependencies}
)

ament_export_libraries(iii_drone_lib)
ament_export_include_directories(include)

install(TARGETS iii_drone_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

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

# Trajectory generator:
custom_cpp_target(trajectory_generator
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/trajectory_generator_node/trajectory_generator_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/trajectory_generator_node/trajectory_interpolator_configuration.hpp"
  "${PROJECT_SOURCE_DIR}/src/control/trajectory_generator_node/trajectory_generator_node.cpp"
)

# Maneuver controller:
custom_cpp_target(maneuver_controller
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/maneuver_controller_node/maneuver_controller_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/maneuver_controller_node/trajectory_generator_client_configuration.hpp"
  "${PROJECT_SOURCE_DIR}/src/control/maneuver_controller_node/maneuver_controller_node.cpp"
)

############
# Install ##
############
# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  "${PROJECT_SOURCE_DIR}/scripts/payload/charger_gripper_node/charger_gripper_node.py"
  DESTINATION lib/${PROJECT_NAME}
)

# Export information to downstream packages
ament_export_dependencies(ament_cmake ${dependencies})

# Install header files
install(DIRECTORY 
  "${PROJECT_SOURCE_DIR}/include/"
  DESTINATION include
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# If building tests, merge the test/launch directory with the main launch directory
if (BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Each test gets its own ROS domain: tests that serve the same services or
  # topics (e.g. /control/trajectory_generator/compute_reference_trajectory)
  # otherwise answer each other when ctest runs them in parallel. Unlike
  # ament_add_ros_isolated_gtest, the runner also isolates when ROS_DOMAIN_ID
  # is inherited (the devcontainer exports ROS_DOMAIN_ID=0) and never picks a
  # live SIM/HIL domain.
  find_package(ament_cmake_ros REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  function(iii_drone_core_add_isolated_gtest target)
    ament_add_gtest("${target}"
      RUNNER "${CMAKE_CURRENT_SOURCE_DIR}/test/ros_domain_isolated_runner.py"
      ${ARGN}
    )
  endfunction()

  ament_add_pytest_test(iii_drone_core_tf_launch_profile_test
    test/test_tf_launch_profile.py
  )

  iii_drone_core_add_isolated_gtest(iii_drone_core_history_test
    test/utils/history_test.cpp
  )

  iii_drone_core_add_isolated_gtest(iii_drone_core_callback_lifetime_test
    test/utils/callback_lifetime_test.cpp
  )
  iii_drone_core_add_isolated_gtest(iii_drone_core_cable_push_profile_test
    test/control/cable_push_profile_test.cpp
  )
  iii_drone_core_add_isolated_gtest(iii_drone_core_hover_thrust_meter_test
    test/control/hover_thrust_meter_test.cpp
  )
  iii_drone_core_add_isolated_gtest(iii_drone_core_multi_threaded_executor_test
    test/utils/multi_threaded_executor_test.cpp
    TIMEOUT 120
  )

  iii_drone_core_add_isolated_gtest(iii_drone_core_math_types_test
    test/utils/math_types_test.cpp
  )

  iii_drone_core_add_isolated_gtest(iii_drone_core_timestamp_test
    test/utils/timestamp_test.cpp
  )

  iii_drone_core_add_isolated_gtest(iii_drone_core_reference_test
    test/control/reference_test.cpp
  )

  iii_drone_core_add_isolated_gtest(iii_drone_core_kinematic_stop_trajectory_test
    test/control/kinematic_stop_trajectory_test.cpp
  )

  iii_drone_core_add_isolated_gtest(iii_drone_core_maneuver_reference_safety_guard_test
    test/control/maneuver_reference_safety_guard_test.cpp
  )
  iii_drone_core_add_isolated_gtest(iii_drone_core_bounded_terminal_position_tracking_test
    test/control/bounded_terminal_position_tracking_test.cpp
  )
  iii_drone_core_add_isolated_gtest(iii_drone_core_measured_odometry_snapshot_test
    test/control/measured_odometry_snapshot_test.cpp
  )
  iii_drone_core_add_isolated_gtest(iii_drone_core_estimated_position_stop_proof_test
    test/control/estimated_position_stop_proof_test.cpp
  )
  iii_drone_core_add_isolated_gtest(iii_drone_core_maneuver_reference_stream_guard_test
    test/control/maneuver_reference_stream_guard_test.cpp
  )
  iii_drone_core_add_isolated_gtest(iii_drone_core_maneuver_reference_client_transaction_test
    test/control/maneuver_reference_client_transaction_test.cpp
  )
  iii_drone_core_add_isolated_gtest(iii_drone_core_reference_callback_token_test
    test/control/reference_callback_token_test.cpp
  )

  iii_drone_core_add_isolated_gtest(iii_drone_core_odometry_ingress_starvation_test
    test/control/odometry_ingress_starvation_test.cpp
  )

  # Several closed-loop cases cover over two minutes of virtual flight in Debug.
  iii_drone_core_add_isolated_gtest(iii_drone_core_trajectory_interpolator_test TIMEOUT 600
    test/control/trajectory_interpolator_test.cpp
  )
  iii_drone_core_add_isolated_gtest(iii_drone_core_trajectory_generator_client_test
    test/control/trajectory_generator_client_test.cpp
  )
  iii_drone_core_add_isolated_gtest(iii_drone_core_trajectory_generator_test
    test/control/trajectory_generator_test.cpp
  )
  iii_drone_core_add_isolated_gtest(iii_drone_core_maneuver_controller_configuration_test
    test/control/maneuver_controller_configuration_test.cpp
  )
  iii_drone_core_add_isolated_gtest(iii_drone_core_waypoint_path_planner_test
    test/control/waypoint_path_planner_test.cpp
  )
  iii_drone_core_add_isolated_gtest(iii_drone_core_follow_waypoint_path_stop_proof_test
    test/control/follow_waypoint_path_stop_proof_test.cpp
  )

  iii_drone_core_add_isolated_gtest(iii_drone_core_cable_aware_trajectory_planner_test
    test/control/cable_aware_trajectory_planner_test.cpp
  )

  iii_drone_core_add_isolated_gtest(iii_drone_core_cable_landing_heading_lock_test
    test/control/cable_landing_heading_lock_test.cpp
  )

  iii_drone_core_add_isolated_gtest(iii_drone_core_basic_adapters_test
    test/adapters/basic_adapters_test.cpp
  )

  iii_drone_core_add_isolated_gtest(iii_drone_core_perception_adapters_test
    test/adapters/perception_adapters_test.cpp
  )

  iii_drone_core_add_isolated_gtest(iii_drone_core_hough_transformer_test
    test/perception/hough_transformer_test.cpp
  )

  iii_drone_core_add_isolated_gtest(iii_drone_core_powerline_direction_test
    test/perception/powerline_direction_test.cpp
  )

  if(TARGET iii_drone_core_callback_lifetime_test)
    target_include_directories(iii_drone_core_callback_lifetime_test PRIVATE ${III_DRONE_INCLUDE_DIRS})
  endif()
  if(TARGET iii_drone_core_cable_push_profile_test)
    target_include_directories(iii_drone_core_cable_push_profile_test PRIVATE ${III_DRONE_INCLUDE_DIRS})
  endif()
  if(TARGET iii_drone_core_hover_thrust_meter_test)
    target_include_directories(iii_drone_core_hover_thrust_meter_test PRIVATE ${III_DRONE_INCLUDE_DIRS})
  endif()
  if(TARGET iii_drone_core_multi_threaded_executor_test)
    target_include_directories(iii_drone_core_multi_threaded_executor_test PRIVATE ${III_DRONE_INCLUDE_DIRS})
    ament_target_dependencies(iii_drone_core_multi_threaded_executor_test rclcpp rclcpp_lifecycle std_msgs)
  endif()
  if(TARGET iii_drone_core_history_test)
    target_include_directories(iii_drone_core_history_test PRIVATE ${III_DRONE_INCLUDE_DIRS})
    target_link_libraries(iii_drone_core_history_test iii_drone_lib)
    ament_target_dependencies(iii_drone_core_history_test ${dependencies})
  endif()

  if(TARGET iii_drone_core_math_types_test)
    target_include_directories(iii_drone_core_math_types_test PRIVATE ${III_DRONE_INCLUDE_DIRS})
    target_link_libraries(iii_drone_core_math_types_test iii_drone_lib)
    ament_target_dependencies(iii_drone_core_math_types_test ${dependencies})
  endif()

  if(TARGET iii_drone_core_timestamp_test)
    target_include_directories(iii_drone_core_timestamp_test PRIVATE ${III_DRONE_INCLUDE_DIRS})
    target_link_libraries(iii_drone_core_timestamp_test iii_drone_lib)
    ament_target_dependencies(iii_drone_core_timestamp_test ${dependencies})
  endif()

  if(TARGET iii_drone_core_reference_test)
    target_include_directories(iii_drone_core_reference_test PRIVATE ${III_DRONE_INCLUDE_DIRS})
    target_link_libraries(iii_drone_core_reference_test iii_drone_lib)
    ament_target_dependencies(iii_drone_core_reference_test ${dependencies})
  endif()

  if(TARGET iii_drone_core_kinematic_stop_trajectory_test)
    target_include_directories(iii_drone_core_kinematic_stop_trajectory_test PRIVATE ${III_DRONE_INCLUDE_DIRS})
    target_link_libraries(iii_drone_core_kinematic_stop_trajectory_test iii_drone_lib)
    ament_target_dependencies(iii_drone_core_kinematic_stop_trajectory_test ${dependencies})
  endif()

  if(TARGET iii_drone_core_maneuver_reference_safety_guard_test)
    target_include_directories(iii_drone_core_maneuver_reference_safety_guard_test PRIVATE ${III_DRONE_INCLUDE_DIRS})
    target_link_libraries(iii_drone_core_maneuver_reference_safety_guard_test iii_drone_lib)
    ament_target_dependencies(iii_drone_core_maneuver_reference_safety_guard_test ${dependencies})
  endif()
  if(TARGET iii_drone_core_bounded_terminal_position_tracking_test)
    target_include_directories(iii_drone_core_bounded_terminal_position_tracking_test PRIVATE ${III_DRONE_INCLUDE_DIRS})
    target_link_libraries(iii_drone_core_bounded_terminal_position_tracking_test iii_drone_lib)
    ament_target_dependencies(iii_drone_core_bounded_terminal_position_tracking_test ${dependencies})
  endif()
  if(TARGET iii_drone_core_measured_odometry_snapshot_test)
    target_include_directories(iii_drone_core_measured_odometry_snapshot_test PRIVATE ${III_DRONE_INCLUDE_DIRS})
    target_link_libraries(iii_drone_core_measured_odometry_snapshot_test iii_drone_lib)
    ament_target_dependencies(iii_drone_core_measured_odometry_snapshot_test ${dependencies})
  endif()
  if(TARGET iii_drone_core_estimated_position_stop_proof_test)
    target_include_directories(iii_drone_core_estimated_position_stop_proof_test PRIVATE ${III_DRONE_INCLUDE_DIRS})
    target_link_libraries(iii_drone_core_estimated_position_stop_proof_test iii_drone_lib)
    ament_target_dependencies(iii_drone_core_estimated_position_stop_proof_test ${dependencies})
  endif()
  if(TARGET iii_drone_core_maneuver_reference_stream_guard_test)
    target_include_directories(iii_drone_core_maneuver_reference_stream_guard_test PRIVATE ${III_DRONE_INCLUDE_DIRS})
    target_link_libraries(iii_drone_core_maneuver_reference_stream_guard_test iii_drone_lib)
    ament_target_dependencies(iii_drone_core_maneuver_reference_stream_guard_test ${dependencies})
  endif()
  if(TARGET iii_drone_core_maneuver_reference_client_transaction_test)
    target_include_directories(iii_drone_core_maneuver_reference_client_transaction_test PRIVATE ${III_DRONE_INCLUDE_DIRS})
    target_link_libraries(iii_drone_core_maneuver_reference_client_transaction_test iii_drone_lib)
    ament_target_dependencies(iii_drone_core_maneuver_reference_client_transaction_test ${dependencies})
  endif()
  if(TARGET iii_drone_core_odometry_ingress_starvation_test)
    target_include_directories(iii_drone_core_odometry_ingress_starvation_test PRIVATE ${III_DRONE_INCLUDE_DIRS})
    target_link_libraries(iii_drone_core_odometry_ingress_starvation_test iii_drone_lib)
    ament_target_dependencies(iii_drone_core_odometry_ingress_starvation_test ${dependencies})
  endif()
  if(TARGET iii_drone_core_reference_callback_token_test)
    target_include_directories(iii_drone_core_reference_callback_token_test PRIVATE ${III_DRONE_INCLUDE_DIRS})
    target_link_libraries(iii_drone_core_reference_callback_token_test iii_drone_lib)
    ament_target_dependencies(iii_drone_core_reference_callback_token_test ${dependencies})
  endif()

  if(TARGET iii_drone_core_trajectory_interpolator_test)
    target_include_directories(iii_drone_core_trajectory_interpolator_test PRIVATE ${III_DRONE_INCLUDE_DIRS})
    target_link_libraries(iii_drone_core_trajectory_interpolator_test iii_drone_lib)
    ament_target_dependencies(iii_drone_core_trajectory_interpolator_test ${dependencies})
  endif()
  if(TARGET iii_drone_core_trajectory_generator_client_test)
    target_include_directories(iii_drone_core_trajectory_generator_client_test PRIVATE ${III_DRONE_INCLUDE_DIRS})
    target_link_libraries(iii_drone_core_trajectory_generator_client_test iii_drone_lib)
    ament_target_dependencies(iii_drone_core_trajectory_generator_client_test ${dependencies})
  endif()
  if(TARGET iii_drone_core_trajectory_generator_test)
    target_include_directories(iii_drone_core_trajectory_generator_test PRIVATE ${III_DRONE_INCLUDE_DIRS})
    target_link_libraries(iii_drone_core_trajectory_generator_test iii_drone_lib)
    ament_target_dependencies(iii_drone_core_trajectory_generator_test ${dependencies})
  endif()
  if(TARGET iii_drone_core_maneuver_controller_configuration_test)
    target_include_directories(iii_drone_core_maneuver_controller_configuration_test PRIVATE ${III_DRONE_INCLUDE_DIRS})
    target_link_libraries(iii_drone_core_maneuver_controller_configuration_test iii_drone_lib)
    ament_target_dependencies(iii_drone_core_maneuver_controller_configuration_test ${dependencies})
  endif()
  if(TARGET iii_drone_core_waypoint_path_planner_test)
    target_include_directories(iii_drone_core_waypoint_path_planner_test PRIVATE ${III_DRONE_INCLUDE_DIRS})
    target_link_libraries(iii_drone_core_waypoint_path_planner_test iii_drone_lib)
    ament_target_dependencies(iii_drone_core_waypoint_path_planner_test ${dependencies})
  endif()
  if(TARGET iii_drone_core_follow_waypoint_path_stop_proof_test)
    target_include_directories(iii_drone_core_follow_waypoint_path_stop_proof_test PRIVATE ${III_DRONE_INCLUDE_DIRS})
    target_link_libraries(iii_drone_core_follow_waypoint_path_stop_proof_test iii_drone_lib)
    ament_target_dependencies(iii_drone_core_follow_waypoint_path_stop_proof_test ${dependencies})
  endif()

  if(TARGET iii_drone_core_cable_aware_trajectory_planner_test)
    target_include_directories(iii_drone_core_cable_aware_trajectory_planner_test PRIVATE ${III_DRONE_INCLUDE_DIRS})
    target_link_libraries(iii_drone_core_cable_aware_trajectory_planner_test iii_drone_lib)
    ament_target_dependencies(iii_drone_core_cable_aware_trajectory_planner_test ${dependencies})
  endif()

  if(TARGET iii_drone_core_cable_landing_heading_lock_test)
    target_include_directories(iii_drone_core_cable_landing_heading_lock_test PRIVATE ${III_DRONE_INCLUDE_DIRS})
    target_link_libraries(iii_drone_core_cable_landing_heading_lock_test iii_drone_lib)
    ament_target_dependencies(iii_drone_core_cable_landing_heading_lock_test ${dependencies})
  endif()

  if(TARGET iii_drone_core_basic_adapters_test)
    target_include_directories(iii_drone_core_basic_adapters_test PRIVATE ${III_DRONE_INCLUDE_DIRS})
    target_link_libraries(iii_drone_core_basic_adapters_test iii_drone_lib)
    ament_target_dependencies(iii_drone_core_basic_adapters_test ${dependencies})
  endif()

  if(TARGET iii_drone_core_perception_adapters_test)
    target_include_directories(iii_drone_core_perception_adapters_test PRIVATE ${III_DRONE_INCLUDE_DIRS})
    target_link_libraries(iii_drone_core_perception_adapters_test iii_drone_lib)
    ament_target_dependencies(iii_drone_core_perception_adapters_test ${dependencies})
  endif()

  if(TARGET iii_drone_core_hough_transformer_test)
    target_include_directories(iii_drone_core_hough_transformer_test PRIVATE ${III_DRONE_INCLUDE_DIRS})
    target_link_libraries(iii_drone_core_hough_transformer_test iii_drone_lib)
    ament_target_dependencies(iii_drone_core_hough_transformer_test ${dependencies})
  endif()

  if(TARGET iii_drone_core_powerline_direction_test)
    target_include_directories(iii_drone_core_powerline_direction_test PRIVATE ${III_DRONE_INCLUDE_DIRS})
    target_link_libraries(iii_drone_core_powerline_direction_test iii_drone_lib)
    ament_target_dependencies(iii_drone_core_powerline_direction_test ${dependencies})
  endif()

  install(DIRECTORY
    test/launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()
