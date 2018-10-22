cmake_minimum_required(VERSION 2.8.3)
project(bringup_dual)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  std_msgs
  nodelet
  tf
)

  add_message_files(
    FILES
    commendMsg.msg
    motorsMsg.msg
    jointstatesMsg.msg
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
  INCLUDE_DIRS 
    include
  LIBRARIES 
    vehicle_v2_nodelet
  CATKIN_DEPENDS 
    rospy 
    roscpp
    geometry_msgs 
    nav_msgs 
    std_msgs 
    ecl_exceptions 
    ecl_sigslots 
    ecl_streams 
    ecl_threads
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(vehicle_controller src/vehicle_controller.cpp)
target_link_libraries(vehicle_controller ${catkin_LIBRARIES})
add_dependencies(vehicle_controller vehicle_control_generate_messages_cpp)

// add_executable(odometry src/odometry.cpp)
// target_link_libraries(odometry ${catkin_LIBRARIES})

