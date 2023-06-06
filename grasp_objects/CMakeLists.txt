cmake_minimum_required(VERSION 3.0.2)
project(grasp_objects)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf
  pcl_ros
  companion_msgs
  kdl_conversions
  nav_msgs
  image_transport
  image_geometry
)
find_package(SuperquadricLib 0.1.0.0 EXACT REQUIRED)


catkin_package(
   INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs companion_msgs kdl_conversions nav_msgs
)



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(constrained_superquadrics_node src/constrained_superquadrics.cpp src/constrained_superquadrics_node.cpp)

add_executable(${PROJECT_NAME}_node src/grasp_objects.cpp src/grasp_objects_node.cpp)


add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(constrained_superquadrics_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  SuperquadricLib::SuperquadricLibModel
)

target_link_libraries(constrained_superquadrics_node
  ${catkin_LIBRARIES}
  SuperquadricLib::SuperquadricLibModel
  stdc++fs
)








