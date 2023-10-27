cmake_minimum_required(VERSION 3.0.2)
project(mot)

set(Eigen3_DIR "/home/fbh/eigen-3.3.7/build")
set(OpenCV_DIR "/home/fbh/opencv-4.1.2/build")
set(cv_bridge_DIR "/home/fbh/Cv_Bridge/install/share/cv_bridge/cmake")

find_package(catkin REQUIRED COMPONENTS
    tf
    roscpp
    Eigen3
    OpenCV
    cv_bridge
    image_transport
    message_filters

    std_msgs
    sensor_msgs
    geometry_msgs
    darknet_ros_msgs
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES mot
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)

add_executable(csrt
    src/csrt.cpp
    src/sortCSRT.cpp
    src/KalmanTracker.cpp
    src/Hungarian.cpp
)

target_link_libraries (csrt
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBRARIES} 
    ${cv_bridge_LIBRARIES}
)