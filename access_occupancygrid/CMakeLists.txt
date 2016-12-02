cmake_minimum_required(VERSION 2.8.3)
project(access_occupancygrid)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(access_occupancygrid_node src/access_occupancygrid.cpp)

target_link_libraries(access_occupancygrid_node
  ${catkin_LIBRARIES}
)
