cmake_minimum_required(VERSION 3.5)

project(template)

set(CMAKE_CXX_STANDARD 11)

# # method 1
# # test with `./tests`
# add_executable(tests ${PROJECT_SOURCE_DIR}/src/main.cc)
# target_link_libraries(tests gtest_main gmock_main pthread)

# # method 2
# # test with `./tests`
# find_package(GTest)
# include_directories(${GTEST_INCLUDE_DIRS})
# add_executable(tests ${PROJECT_SOURCE_DIR}/src/main.cc)
# target_link_libraries(tests ${GTEST_BOTH_LIBRARIES} pthread)
# # target_link_libraries(tests GTest::GTest GTest::Main pthread)

# method 3
# test with `cmake -S . -B build && cmake --build build && cd build && ctest`
enable_testing()
find_package(GTest REQUIRED)
add_executable(tests ${PROJECT_SOURCE_DIR}/src/main.cc)
target_link_libraries(tests ${GTEST_BOTH_LIBRARIES} pthread)
include(GoogleTest)
gtest_discover_tests(tests)
