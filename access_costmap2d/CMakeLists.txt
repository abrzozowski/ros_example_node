cmake_minimum_required(VERSION 2.8.3)
project(access_costmap2d)

find_package(catkin REQUIRED COMPONENTS
  tf 
  costmap_2d
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(access_costmap2d_node src/access_costmap2d.cpp)

target_link_libraries(access_costmap2d_node
  ${catkin_LIBRARIES}
)
