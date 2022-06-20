cmake_minimum_required(VERSION 3.5)
project(fiborobotlab2)

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

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  config
  rviz
  urdf
  DESTINATION share/${PROJECT_NAME}
  )
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
"msg/Rstate.msg")
install(PROGRAMS
  scripts/test/Dynamixel_test.py
  scripts/test/pose_cam_aruco.py
  scripts/Detection_Node.py
  scripts/detect.py
  scripts/test/Client_position.py
  scripts/PID_Node.py
  scripts/pid_control.py
  scripts/State_Controll.py
  scripts/confirmed_node.py
  scripts/Follow_ball.py
  DESTINATION lib/${PROJECT_NAME}
  )
  
ament_package()
