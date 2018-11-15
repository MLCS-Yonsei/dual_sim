cmake_minimum_required(VERSION 2.8.3)
project(bringup_dual)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  nodelet
  tf
)

  generate_messages(
    DEPENDENCIES
     std_msgs
  )


catkin_package(
  LIBRARIES 
    bringup_dual
  CATKIN_DEPENDS 
    geometry_msgs 
    nav_msgs 
    roscpp 
    std_msgs
)

catkin_package(
  LIBRARIES 
    vehicle_v2_nodelet
  CATKIN_DEPENDS 
    rospy 
    roscpp
    geometry_msgs 
    nav_msgs 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)