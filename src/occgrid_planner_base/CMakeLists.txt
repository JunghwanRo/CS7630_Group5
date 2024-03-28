cmake_minimum_required(VERSION 3.8)
project(occgrid_planner_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -g -ggdb -O3)
endif()

# find dependencies
find_package(OpenCV REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cs7630_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "OpenCV includes: '${OpenCV_INCLUDE_DIRS}'")

add_executable(occgrid_planner_base src/occgrid_planner.cpp)
target_include_directories(occgrid_planner_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(occgrid_planner_base ${OpenCV_LIBRARIES})
target_compile_features(occgrid_planner_base PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  occgrid_planner_base
  "rclcpp" "cs7630_msgs" "sensor_msgs" "std_msgs" "nav_msgs" "geometry_msgs" "tf2_geometry_msgs" "tf2_ros"
)

add_executable(path_optimizer_base src/path_optimizer.cpp)
target_include_directories(path_optimizer_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(path_optimizer_base PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  path_optimizer_base
  "rclcpp" "cs7630_msgs" "sensor_msgs" "std_msgs" "nav_msgs" "geometry_msgs" "tf2_geometry_msgs" "tf2_ros"
)

add_executable(path_follower_base src/path_follower.cpp)
target_include_directories(path_follower_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(path_follower_base PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  path_follower_base
  "rclcpp" "cs7630_msgs" "sensor_msgs" "std_msgs" "nav_msgs" "geometry_msgs" "tf2_geometry_msgs" "tf2_ros"
)

install(TARGETS occgrid_planner_base path_optimizer_base path_follower_base
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch/
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
