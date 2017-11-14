cmake_minimum_required(VERSION 2.8.3)
project(map_sim)
set(CMAKE_CXX_STANDARD 11)

## ROS library dependencies
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    geometry_msgs
    tf2
    tf2_ros
)
catkin_package(
    CATKIN_DEPENDS roscpp geometry_msgs rospy tf2 tf2_ros
)

###########
## Build ##
###########

## System libary dependencies

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

set(SYSTEM_LIBRARIES
)

## Declare a C++ executable
add_executable(markers "src/sim/markers.cpp")
add_executable(quad "src/sim/quad.cpp")
add_executable(bots "src/sim/bots.cpp")

## Add cmake target dependencies of the executable
add_dependencies(markers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(quad ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(bots ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(markers ${catkin_LIBRARIES} ${SYSTEM_LIBRARIES})
target_link_libraries(quad ${catkin_LIBRARIES} ${SYSTEM_LIBRARIES})
target_link_libraries(bots ${catkin_LIBRARIES} ${SYSTEM_LIBRARIES})
