cmake_minimum_required(VERSION 3.0.2)
project(synchronizer_example)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  custom_msg_example
  cv_bridge
  image_transport
  message_filters
  roscpp
  std_msgs
)

find_package(OpenCV 4 REQUIRED)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(synchronizer_demo src/synchronizer_demo.cc)
target_link_libraries(synchronizer_demo ${catkin_LIBRARIES} ${OpenCV_LIBS})