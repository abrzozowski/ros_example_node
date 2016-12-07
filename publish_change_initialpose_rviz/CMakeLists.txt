cmake_minimum_required(VERSION 2.8.3)
project(publish_change_initialpose_rviz)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    tf
)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(publish_change_initialpose_rviz_node
    src/publish_change_initialpose_rviz.cpp
)
target_link_libraries(publish_change_initialpose_rviz_node
    ${catkin_LIBRARIES}
)