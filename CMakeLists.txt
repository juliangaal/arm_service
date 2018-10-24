cmake_minimum_required(VERSION 2.8.3)
project(arm_service)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  anchor_msgs
  jaco_manipulation
  message_generation
  geometry_msgs
  roscpp
  rospy
  std_msgs
  actionlib
  actionlib_msgs
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   ArmInstruction.srv
   AnchorVisuals.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   anchor_msgs
   std_msgs
   geometry_msgs
 )

catkin_package(
  LIBRARIES arm_service
  CATKIN_DEPENDS anchor_msgs jaco_manipulation message_generation roscpp rospy std_msgs geometry_msgs
)

### Declare a C++ library
add_library(${PROJECT_NAME} src/arm_service.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_library(visuals_updater src/visuals/visuals_updater.cpp)
target_include_directories(visuals_updater PUBLIC include ${catkin_INCLUDE_DIRS})
add_dependencies(visuals_updater ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node src/arm_service_node.cpp)
target_include_directories(${PROJECT_NAME}_node PUBLIC include ${catkin_INCLUDE_DIRS})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node arm_service visuals_updater ${catkin_LIBRARIES})
