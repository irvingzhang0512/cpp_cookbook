cmake_minimum_required(VERSION 3.0.2)
project(hello_world)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

add_executable(hello_world src/hello_world.cc)
target_link_libraries(hello_world ${catkin_LIBRARIES})
