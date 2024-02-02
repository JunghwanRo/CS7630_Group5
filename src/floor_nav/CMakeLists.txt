cmake_minimum_required(VERSION 3.8)
project(floor_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -std=c++17 -g -ggdb -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(task_manager_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(task_manager_lib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(topic_tools_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)

find_package(PCL REQUIRED QUIET COMPONENTS common io)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(task_manager_nav2)

include_directories(
  include
  ${PCL_COMMON_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_action_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${topic_tools_interfaces_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
  ${task_manager_msgs_INCLUDE_DIRS}
  ${custom_interfaces_INCLUDE_DIRS}
  ${task_manager_lib_INCLUDE_DIRS}
  ${task_manager_nav2_INCLUDE_DIRS}
  ${kobuki_ros_interfaces_INCLUDE_DIRS}
)

SET(ALL_LIBS 
    ${PCL_LIBRARIES} 
    ${pcl_conversions_LIBRARIES}
    ${rclcpp_LIBRARIES} 
    ${rclcpp_action_LIBRARIES} 
    ${task_manager_lib_LIBRARIES} 
    ${topic_tools_interfaces_LIBRARIES} 
    ${sensor_msgs_LIBRARIES} 
    ${geometry_msgs_LIBRARIES} 
    ${tf2_LIBRARIES} 
    ${tf2_geometry_msgs_LIBRARIES} 
    ${task_manager_msgs_LIBRARIES} 
    ${custom_interfaces_LIBRARIES}
    ${std_msgs_LIBRARIES} 
    ${std_srvs_LIBRARIES} 
    ${Boost_THREAD_LIBRARIES} 
    ${CMAKE_DL_LIBS})

file(SHA256 ${PROJECT_SOURCE_DIR}/include/floor_nav/SimTasksEnv.h ENV_CHECK_SUM)
add_compile_definitions(ENV_CHECK_SUM=${ENV_CHECK_SUM})

ADD_EXECUTABLE( floornav_task_server src/task_server.cpp src/SimTasksEnv.cpp) 
ament_target_dependencies(floornav_task_server task_manager_lib
    task_manager_msgs geometry_msgs rclcpp rclcpp_action std_msgs task_manager_nav2
    tf2 tf2_geometry_msgs custom_interfaces)
# Necessary target for the tasks to see the symbols of the main
TARGET_LINK_OPTIONS(floornav_task_server PRIVATE "-rdynamic")
TARGET_LINK_LIBRARIES(floornav_task_server ${ALL_LIBS})

ADD_LIBRARY( floornav_TaskNav2 SHARED tasks/TaskNav2.cpp)
TARGET_LINK_LIBRARIES(floornav_TaskNav2 ${ALL_LIBS}
    ${task_manager_nav2_LIBRARIES} )


ADD_LIBRARY( floornav_TaskSetMux SHARED tasks/TaskSetMux.cpp)
TARGET_LINK_LIBRARIES(floornav_TaskSetMux ${ALL_LIBS})

ADD_LIBRARY( floornav_TaskTrigger SHARED tasks/TaskTrigger.cpp)
TARGET_LINK_LIBRARIES(floornav_TaskTrigger ${ALL_LIBS})

ADD_LIBRARY( floornav_TaskConstant SHARED tasks/TaskConstant.cpp)
TARGET_LINK_LIBRARIES(floornav_TaskConstant ${ALL_LIBS})

ADD_LIBRARY( floornav_TaskGoTo SHARED tasks/TaskGoTo.cpp)
TARGET_LINK_LIBRARIES(floornav_TaskGoTo ${ALL_LIBS})

ADD_LIBRARY( floornav_TaskGoToPose SHARED tasks/TaskGoToPose.cpp)
TARGET_LINK_LIBRARIES(floornav_TaskGoToPose ${ALL_LIBS})

ADD_LIBRARY( floornav_TaskSetHeading SHARED tasks/TaskSetHeading.cpp)
TARGET_LINK_LIBRARIES(floornav_TaskSetHeading ${ALL_LIBS})

ADD_LIBRARY( floornav_TaskStareAtFace SHARED tasks/TaskStareAtFace.cpp)
TARGET_LINK_LIBRARIES(floornav_TaskStareAtFace ${ALL_LIBS})

ADD_LIBRARY( floornav_TaskPlanTo SHARED tasks/TaskPlanTo.cpp)
TARGET_LINK_LIBRARIES(floornav_TaskPlanTo ${ALL_LIBS})

ADD_LIBRARY( floornav_TaskWaitForROI SHARED tasks/TaskWaitForROI.cpp)
TARGET_LINK_LIBRARIES(floornav_TaskWaitForROI ${ALL_LIBS})

ADD_LIBRARY( floornav_TaskWaitForFace SHARED tasks/TaskWaitForFace.cpp)
TARGET_LINK_LIBRARIES(floornav_TaskWaitForFace ${ALL_LIBS})

ADD_LIBRARY( floornav_TaskWander SHARED tasks/TaskWander.cpp)
TARGET_LINK_LIBRARIES(floornav_TaskWander ${ALL_LIBS})

ADD_LIBRARY( floornav_TaskAutoDock SHARED tasks/TaskAutoDock.cpp)
TARGET_LINK_LIBRARIES(floornav_TaskAutoDock ${ALL_LIBS}
    ${kobuki_ros_interfaces_LIBRARIES})

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

install(TARGETS
  floornav_task_server
  DESTINATION lib/${PROJECT_NAME})


install(TARGETS
    floornav_TaskAutoDock
    floornav_TaskWaitForROI
    floornav_TaskWaitForFace
    floornav_TaskWander
    floornav_TaskSetHeading
    floornav_TaskStareAtFace
    floornav_TaskPlanTo
    floornav_TaskGoTo
    floornav_TaskGoToPose
    floornav_TaskConstant
    floornav_TaskTrigger
    floornav_TaskSetMux
    floornav_TaskNav2
    DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)


install(PROGRAMS  
	missions/graph_forever.py
	missions/test_goto.py
	missions/test_graph_2d.py
	missions/test_graph.py
	missions/test_graph-test.py
	missions/test_planto.py
	missions/test_set_heading.py
	missions/test_wander.py
  missions/people_finder.py
  missions/gotopose.py
    DESTINATION lib/${PROJECT_NAME} 
)
ament_package()