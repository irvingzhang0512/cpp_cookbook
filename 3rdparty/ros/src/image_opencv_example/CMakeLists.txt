cmake_minimum_required(VERSION 3.0.2)
project(image_opencv_example)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
)

find_package(OpenCV 4 REQUIRED)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(image_publisher src/image_publisher.cc)
target_link_libraries(image_publisher ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(image_subscriber src/image_subscriber.cc)
target_link_libraries(image_subscriber ${catkin_LIBRARIES} ${OpenCV_LIBS})
