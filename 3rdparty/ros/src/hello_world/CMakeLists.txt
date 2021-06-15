cmake_minimum_required(VERSION 3.0.2)
project(hello_world)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

add_executable(main src/main.cc)
target_link_libraries(main ${catkin_LIBRARIES})
