cmake_minimum_required(VERSION 3.5)

project(cpp_cookbook_3rdparty)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
MESSAGE(${OpenCV_INCLUDE_DIRS})
MESSAGE(${OpenCV_LIBS})

add_executable(test_opencv ${PROJECT_SOURCE_DIR}/src/opencv_cookbook.cc)
target_link_libraries(test_opencv ${OpenCV_LIBS})

# need to install freetype first
add_executable(test_opencv_freetype ${PROJECT_SOURCE_DIR}/src/opencv_freetype_cookbook.cc)
target_link_libraries(test_opencv_freetype ${OpenCV_LIBS} )
