cmake_minimum_required(VERSION 2.8.3)
project(access_occupancygrid_by_service)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(access_occupancygrid_by_service_node src/access_occupancygrid_by_service.cpp)

target_link_libraries(access_occupancygrid_by_service_node
  ${catkin_LIBRARIES}
)
