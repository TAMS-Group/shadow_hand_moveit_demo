cmake_minimum_required(VERSION 2.8.3)
project(hand_demo)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  roscpp
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/hand_demo.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_warehouse src/hand_demo_warehouse.cpp)
add_dependencies(${PROJECT_NAME}_warehouse ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_warehouse ${catkin_LIBRARIES})
