cmake_minimum_required(VERSION 2.8.3)
project(hand_demo)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  roscpp
  message_generation
  cv_bridge
)

add_message_files(
  FILES
  NamedRobotPose.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
)
catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/hand_demo.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_warehouse_onecamera src/hand_demo_warehouse_onecamera.cpp)
add_dependencies(${PROJECT_NAME}_warehouse_onecamera ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_warehouse_onecamera ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_warehouse_onecamera_video src/hand_demo_warehouse_onecamera_video.cpp)
add_dependencies(${PROJECT_NAME}_warehouse_onecamera_video ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_warehouse_onecamera_video ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_warehouse_onecamera_conti src/hand_demo_warehouse_onecamera_conti.cpp)
add_dependencies(${PROJECT_NAME}_warehouse_onecamera_conti ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_warehouse_onecamera_conti ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_warehouse_twocamera src/hand_demo_warehouse_twocamera.cpp)
add_dependencies(${PROJECT_NAME}_warehouse_twocamera ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_warehouse_twocamera ${catkin_LIBRARIES})

add_executable(save_data src/save_data.cpp)
add_dependencies(save_data ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(save_data ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_random_twocamera src/hand_demo_random_twocamera.cpp)
add_dependencies(${PROJECT_NAME}_random_twocamera ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_random_twocamera ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_random_onecamera src/hand_demo_random_onecamera.cpp)
add_dependencies(${PROJECT_NAME}_random_onecamera ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_random_onecamera ${catkin_LIBRARIES})
