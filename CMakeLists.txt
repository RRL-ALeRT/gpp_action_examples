cmake_minimum_required(VERSION 3.8)
project(gpp_action_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gpp_action_examples_interface REQUIRED)

add_executable(spot_body_pose cpp/spot_body_pose.cpp)
ament_target_dependencies(spot_body_pose rclcpp std_msgs geometry_msgs gpp_action_examples_interface)

add_executable(vision_start_frame_tf2 cpp/vision_start_frame_tf2.cpp)
ament_target_dependencies(vision_start_frame_tf2 rclcpp std_msgs geometry_msgs gpp_action_examples_interface tf2 tf2_ros)


install(TARGETS
  spot_body_pose
  vision_start_frame_tf2
  DESTINATION lib/${PROJECT_NAME}
)

set(PYTHON_NODES
  audio_action_server.py
  frontier_exploration_action_server.py
  print_srv_server.py
  gpp_trajectory.py
)
install(DIRECTORY python/${PROJECT_NAME}
  DESTINATION lib
)
foreach(PY_FILE ${PYTHON_NODES})
  execute_process(COMMAND chmod +x ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/${PY_FILE})
endforeach()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY sounds
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
