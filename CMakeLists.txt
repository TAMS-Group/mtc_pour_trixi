cmake_minimum_required(VERSION 3.0.2)
project(mtc_pour_trixi)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
	actionlib
	moveit_ros_planning_interface
	moveit_task_constructor_core
	moveit_task_constructor_msgs
	mtc_pour
	roscpp
	tf
	)

catkin_package()

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(trixi_pick_pour_place src/trixi_pick_pour_place.cpp)
add_dependencies(trixi_pick_pour_place ${catkin_EXPORTED_TARGETS})

target_link_libraries(trixi_pick_pour_place
  ${catkin_LIBRARIES}
)
