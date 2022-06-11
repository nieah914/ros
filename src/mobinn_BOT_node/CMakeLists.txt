cmake_minimum_required(VERSION 3.0.2)
project(mobinn_BOT_node)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS serial std_msgs
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


add_executable(mobinn_BOT_node src/mobinn_BOT_node.cpp)


target_link_libraries(mobinn_BOT_node
   ${catkin_LIBRARIES}
 )


