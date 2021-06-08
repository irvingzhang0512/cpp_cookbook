cmake_minimum_required(VERSION 3.5)

project(template)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(ffmpeg_h264 ${PROJECT_SOURCE_DIR}/src/ffmpeg_h264_cookbook.cc ${PROJECT_SOURCE_DIR}/src/h264decoder.cc ${PROJECT_SOURCE_DIR}/src/h264encoder.cc)
target_link_libraries(ffmpeg_h264 ${OpenCV_LIBS} avformat avdevice  avcodec avutil  avfilter postproc  swresample swscale)
