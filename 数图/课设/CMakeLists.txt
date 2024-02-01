cmake_minimum_required(VERSION 3.0.2)
project(exp4)
find_package(catkin REQUIRED COMPONENTS
roscpp
std_msgs
geometry_msgs
sensor_msgs
cv_bridge
)
find_package(OpenCV REQUIRED)
catkin_package()
include_directories(
${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
)
add_executable(exp4 src/exp4.cpp)
add_executable(demo src/demo.cpp)
target_link_libraries(exp4
${catkin_LIBRARIES}
${OpenCV_LIBRARIES}
)
target_link_libraries(demo
${catkin_LIBRARIES}
${OpenCV_LIBRARIES}
)
