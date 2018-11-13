cmake_minimum_required(VERSION 2.8.3)
project(tejas)

add_compile_options(-std=c++11 -O3 -DUSE_AVX_INSTRUCTIONS=ON -fopenmp) # -DCMAKE_BUILD_TYPE=Release
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	std_msgs
	roscpp
	sensor_msgs
	geometry_msgs
	message_generation
	tf
)

catkin_package(
		CATKIN_DEPENDS
		message_runtime
		actionlib_msgs
)


include_directories(
	${catkin_INCLUDE_DIRS}
)


add_executable(velocity_test src/velocity_test.cpp)
add_dependencies(velocity_test ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(velocity_test
		${catkin_LIBRARIES}
)

add_executable(position_test src/position_test.cpp)
add_dependencies(position_test ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(position_test
		${catkin_LIBRARIES}
)

add_executable(robot_control2 src/robot_control.cpp)
add_dependencies(robot_control2 ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(robot_control2
		${catkin_LIBRARIES}
)

add_executable(force_reader src/force_reader.cpp)
add_dependencies(force_reader ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(force_reader
		${catkin_LIBRARIES}
)

add_executable(tf_caster src/tf_caster.cpp)
add_dependencies(tf_caster ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(tf_caster
		${catkin_LIBRARIES}
)
