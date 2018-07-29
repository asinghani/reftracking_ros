cmake_minimum_required(VERSION 2.8.3)
project(reftracking_ros)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    roscpp
    std_msgs
    pathfinder_ros
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(reftracking_node src/node.cpp)
target_link_libraries(reftracking_node ${catkin_LIBRARIES})
add_dependencies(reftracking_node ${catkin_EXPORTED_TARGETS})
