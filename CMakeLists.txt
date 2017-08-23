cmake_minimum_required(VERSION 2.8.3)
project(qrcode)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
)

generate_messages(DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  sensor_msgs
)

include_directories(
# include
${catkin_INCLUDE_DIRS}
)
include_directories(/home/ubuntu/catkin_ws/devel/include/ARToolKitPlus)

add_executable(qr_detection_single
  src/qr_detection_single.cpp
)

target_link_libraries(qr_detection_single
  ${catkin_LIBRARIES}
)
target_link_libraries(qr_detection_single /home/ubuntu/catkin_ws/devel/lib/ARToolKitPlus/libARToolKitPlus.so)
add_executable(camera
  src/camera.cpp
)
target_link_libraries(camera
  ${catkin_LIBRARIES}
)


