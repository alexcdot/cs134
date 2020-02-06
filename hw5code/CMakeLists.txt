cmake_minimum_required(VERSION 2.8.3)
project(hw5code)

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
  ThrowCommand.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
#   sensor_msgs
#   std_msgs
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
add_executable(fkinnode  src/fkinnode.cpp)

## Add cmake target dependencies of the executables
add_dependencies(fkinnode  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(fkinnode  ${catkin_LIBRARIES})
