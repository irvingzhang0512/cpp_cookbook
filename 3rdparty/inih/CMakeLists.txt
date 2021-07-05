cmake_minimum_required(VERSION 3.5)

project(template)

set(CMAKE_CXX_STANDARD 11)

# add inih
include_directories(${PROJECT_SOURCE_DIR}/include)
add_library(inih ${PROJECT_SOURCE_DIR}/src/ini.c ${PROJECT_SOURCE_DIR}/src/INIReader.cpp)

add_executable(test_main ${PROJECT_SOURCE_DIR}/src/main.cc)
target_link_libraries(test_main inih)
