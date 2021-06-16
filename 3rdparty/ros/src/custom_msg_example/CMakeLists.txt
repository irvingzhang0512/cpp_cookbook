cmake_minimum_required(VERSION 3.0.2)
project(custom_msg_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  # add the following line
  message_generation
)

add_message_files(
  FILES
  # add the following file
  can_data.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# add_executable(publisher src/publisher.cc)
# target_link_libraries(publisher ${catkin_LIBRARIES})

# add_executable(subscriber src/subscriber.cc)
# target_link_libraries(subscriber ${catkin_LIBRARIES})
