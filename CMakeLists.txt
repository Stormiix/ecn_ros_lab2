cmake_minimum_required(VERSION 2.8.3)
project(puppet)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(puppet_node src/puppet_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(puppet_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############
install(TARGETS puppet_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
