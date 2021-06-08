cmake_minimum_required(VERSION 3.5)

project(x264)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(test_x264 ${PROJECT_SOURCE_DIR}/src/x264_cookbook.cc ${PROJECT_SOURCE_DIR}/src/x264_encoder.cc)
target_link_libraries(test_x264 x264)
target_link_libraries(test_x264 ${OpenCV_LIBS})
