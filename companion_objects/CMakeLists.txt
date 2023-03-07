cmake_minimum_required(VERSION 3.0.2)
project(companion_objects)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gazebo_msgs
)

install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds)
install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models)

catkin_package(

)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

