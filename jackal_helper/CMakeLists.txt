cmake_minimum_required(VERSION 3.16.3)
project(jackal_helper)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(CATKIN_PACKAGE_BIN_DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/bin)

## Find catkin macros and libraries
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
    pkg_check_modules(GAZEBO gazebo)
endif()
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
    geometry_msgs
)

include(FindBoost)
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)
find_package(Protobuf REQUIRED)

catkin_python_setup()
catkin_install_python(PROGRAMS scripts/obstacle_avoidance_node.py 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_package(
    CATKIN_DEPENDS gazebo_msgs message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

## Declare a C++ executable
add_executable(collision_publisher_node src/collision_publisher.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(collision_publisher_node
   ${catkin_LIBRARIES}
)

target_link_libraries(collision_publisher_node ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} pthread)
