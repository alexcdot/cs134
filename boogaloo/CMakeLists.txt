cmake_minimum_required(VERSION 2.8.3)
project(boogaloo)

## Set the package dependencies: Find catkin macros/libraries and
## other required packages.
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  sensor_msgs
  message_generation
)

find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  JointCommand.msg
  PoseCommand.msg
  ThrowCommand.msg
  RobotState.msg
  MassChange.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

## Catkin configuration.  We don't have dependent projects.
catkin_package(
  CATKIN_DEPENDS message_runtime
)


###########
## Build ##
###########

## Specify locations of header files
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

## Declare the C++ executables
add_executable(armcontroller  
  src/armcontroller.hpp src/armcontroller.cpp
  src/common.hpp
  src/splinemanager.hpp src/splinemanager.cpp
  src/kinematics.hpp src/kinematics.cpp)

## Add cmake target dependencies of the executables
add_dependencies(armcontroller  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(armcontroller  ${catkin_LIBRARIES})
