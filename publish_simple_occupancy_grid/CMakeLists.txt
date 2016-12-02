cmake_minimum_required(VERSION 2.8.3)
project(publish_simple_occupancy_grid)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES publish_simple_occupancy_grid
 CATKIN_DEPENDS nav_msgs roscpp
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(publish_simple_occupancy_grid_node src/publish_simple_occupancy_grid.cpp)

target_link_libraries(publish_simple_occupancy_grid_node
  ${catkin_LIBRARIES}
)