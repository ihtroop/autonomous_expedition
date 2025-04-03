cmake_minimum_required(VERSION 3.8)
project(autonomous_stack)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)  # Added this as it was missing but needed for C++ nodes

# Include directories
include_directories(include)

# Add cpp executables
add_executable(obstacle_detection src/obstacle_detection.cpp)
ament_target_dependencies(obstacle_detection
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

add_executable(pit_detection src/pit_detection.cpp)
ament_target_dependencies(pit_detection
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

# Install cpp executables
install(TARGETS
  obstacle_detection
  pit_detection
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts - corrected RENAME syntax
install(PROGRAMS
  scripts/input_one_with_perfect_arrow.py
  RENAME input1a
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/input_one_with_imperfect_arrow.py
  RENAME input1b
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/input_two.py
  RENAME input2
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/controller.py
  RENAME controller
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/obstacle_avoidance.py
  RENAME avoidance
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/map.py
  RENAME map
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
