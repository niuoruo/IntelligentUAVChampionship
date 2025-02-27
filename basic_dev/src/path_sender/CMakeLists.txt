cmake_minimum_required(VERSION 3.0.2)
project(path_sender)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
airsim_ros
message_generation
nav_msgs
# nodelet
roscpp
rospy
sensor_msgs
std_msgs
geographic_msgs
geometry_msgs
std_srvs
tf2
tf2_ros
tf2_sensor_msgs
tf2_geometry_msgs
image_transport
OpenCV
cv_bridge
pcl_ros
pcl_conversions
)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
add_message_files(
  FILES
  WayPoints.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime roscpp std_msgs # nodelet 
)
find_package(OpenCV4 )

include_directories(include ${catkin_INCLUDE_DIRS}  ${OpenCv_INCLUDE_DIRS})


add_executable(path_sender src/path_sender.cpp )
target_link_libraries(path_sender yaml-cpp ${catkin_LIBRARIES} ${OpenCv_LIBS})

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  # PATTERN ".git" EXCLUDE
)