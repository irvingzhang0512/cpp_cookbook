cmake_minimum_required(VERSION 3.5)

project(template)

set(CMAKE_CXX_STANDARD 11)

find_package(spdlog REQUIRED)

add_executable(test_spdlog ${PROJECT_SOURCE_DIR}/src/spdlog_cookbook.cc)
target_link_libraries(test_spdlog PRIVATE spdlog::spdlog)
