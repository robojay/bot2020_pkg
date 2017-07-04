cmake_minimum_required(VERSION 2.8.3)
project(bot2020_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

find_package(Boost REQUIRED COMPONENTS system)



## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs  
 )

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(bot2020_node src/bot2020_node.cpp)

target_link_libraries(bot2020_node
   ${catkin_LIBRARIES}
 )

