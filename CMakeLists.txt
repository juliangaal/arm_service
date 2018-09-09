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

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES arm_service
  CATKIN_DEPENDS anchor_msgs jaco_manipulation message_generation roscpp rospy std_msgs geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

### Declare a C++ library
add_library(${PROJECT_NAME}
  src/arm_service.cpp
)

add_library(visuals_updater
            src/visuals/visuals_updater.cpp
)

### Add cmake target dependencies of the library
### as an example, code may need to be generated before libraries
### either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(visuals_updater ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/arm_service_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES})

target_link_libraries(${PROJECT_NAME}_node
  arm_service
  visuals_updater
  ${catkin_LIBRARIES})
