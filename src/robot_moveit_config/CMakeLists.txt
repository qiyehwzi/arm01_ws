cmake_minimum_required(VERSION 3.1.3)
project(robot_moveit_config)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  actionlib
  actionlib_msgs
  serial
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2_geometry_msgs
  tf2_ros
)

catkin_package(
	CATKIN_DEPENDS roscpp std_msgs   actionlib   actionlib_msgs serial
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2_geometry_msgs
  tf2_ros
)

include_directories(include ${catkin_INCLUDE_DIRS})

include_directories(include )

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(serial_receiver src/serial_receiver.cpp)
target_link_libraries(serial_receiver ${catkin_LIBRARIES})
add_dependencies(serial_receiver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(serial_sender src/serial_sender.cpp)
target_link_libraries(serial_sender ${catkin_LIBRARIES})

add_executable(trajectory_plan_z_down src/trajectory_plan_z_down.cpp)
target_link_libraries(trajectory_plan_z_down ${catkin_LIBRARIES})

add_executable(trajectory_plan_z_up src/trajectory_plan_z_up.cpp)
target_link_libraries(trajectory_plan_z_up ${catkin_LIBRARIES})

add_executable(moveit_planning_scene_demo src/moveit_planning_scene_demo.cpp)
target_link_libraries(moveit_planning_scene_demo ${catkin_LIBRARIES} rt pthread)

# add_executable(share_memory src/share_memory.cpp)
# target_link_libraries(share_memory ${catkin_LIBRARIES} rt pthread)

