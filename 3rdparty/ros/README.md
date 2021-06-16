# ROS cookbook

## Preview

+ Need to install ros first.
+ Example List
  + [Hello World Example](#hello-world-example): What you need to do to create a ROS app.
  + [Publisher and Subscribe](#publisher-and-subscriber): String demo for publisher and subsciber.
  + 

## Hello World Example

+ Init workspace:

```shell
# /path/to/cpp_cookbook/3rdparty/ros is empty
cd /path/to/cpp_cookbook/3rdparty/ros

mkdir src && cd src

# the generated CMakeLists.txt is used for pkg related configs
catkin_init_workspace

# compile whole workspace
cd ..
catkin_make

# update environment configs
source devel/setup.bash
```

+ Init pkg

```shell
cd /path/to/cpp_cookbook/3rdparty/ros

cd src
# the name of this pkg is `hello_world`, create a folder in `src/hello_world`
# and two files `CMakeLists.txt` and `package.xml` are created
catkin_create_pkg hello_world
```

+ Add Hello World code in pkg, `src/hello_world/src/main.cc`

```cpp
#include "ros/ros.h"

int main(int argc,char **argv) 
{
  // name of this node is "hello_node"
  ros::init(argc,argv,"hello_node");
  ros::NodeHandle nh;
  // print string with roscpp function ORS_INFO_STREAM
  ROS_INFO_STREAM("hello world!!!");
}
```

+ Modify pkg cmake config, aka `src/hello_world/CMakeLists.txt` and `src/hello_world/package.xml`

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(hello_world)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

add_executable(main src/main.cc)
target_link_libraries(main ${catkin_LIBRARIES})

```

```xml
<buildtool_depend>catkin</buildtool_depend>
<build_depend>roscpp</build_depend>
<build_export_depend>roscpp</build_export_depend>
<exec_depend>roscpp</exec_depend>
```

+ Compile and run
  + Compile
    + Compile the whole workspace: `cd /path/to/cpp_cookbook/3rdparty/ros && catkin_make`
    + Compile single pkg: `cd /path/to/cpp_cookbook/3rdparty/ros && catkin_make -DCATKIN_WHITELIST_PACKAGES="hello_world"`
  + Run:
    + Terminal No.1: `roscore`
    + Terminal No.2:

```shell
cd /path/to/cpp_cookbook/3rdparty/ros
source devel/setup.bash
# where `hello_world` is from `catkin_create_pkg hello_world`
# `main` is from `add_executable(main src/main.cpp)`
rosrun hello_world main
```

## Publisher and Subscriber

+ Use `Publisher` to publish string messages to topic `chatter` and `Subscriber` to receive messages from the same topic.

```shell
cd /path/to/cpp_cookbook/3rdparty/ros

# 1. Make sure workspace is probably created.
# 2. Create pkg with dependencies `roscpp` and `std_msgs`
#   Please note that, we don't need to add dependencies manually.
cd src
catkin_create_pkg topic_example roscpp std_msgs
cd ..

# 3. Create publish_node.cc and subscribe_node.cc in src/topic_example/src
# 4. Compile pkg
catkin_make -DCATKIN_WHITELIST_PACKAGES="topic_example"

# 5. Run
# Terminal No.1, roscore
roscore
# Terminal No.2, publisher
cd /path/to/cpp_cookbook/3rdparty/ros
source devel/setup.bash
rosrun topic_example publish_node 
# Terminal No.3, subscriber
cd /path/to/cpp_cookbook/3rdparty/ros
source devel/setup.bash
rosrun topic_example subscribe_node 
```

## Create Custom Message

+ Create a custom message `can_data.msg`, and create a publisher and a subscriber to send/receive this kind of message.

```shell
cd /path/to/cpp_cookbook/3rdparty/ros

# 1. Make sure workspace is probably created.
# 2. Create pkg with dependencies `roscpp` and `std_msgs`
cd src
catkin_create_pkg custom_msg_example roscpp std_msgs
cd ..

# 3. Create msg file `can_data.msg` in `src/custom_msg_example/msg`
# 4. Modify related configs(List custom msg related configs)
# 4.1. package.xml
#   Uncomment the following lines
#     <build_depend>message_generation</build_depend>
#     <build_export_depend>message_generation</build_export_depend>
#     <exec_depend>message_runtime</exec_depend>
# 4.2 CMakeLists.txt
#   Add `message_generation` in find_pakcage
#   Add `xxx.msg` in add_message_files
#   Uncomment `generate_messages`
#   Add `message_runtime` in catkin_package

# 5. Create Publisher and Subscriber
# 6. Compile and run
catkin_make -DCATKIN_WHITELIST_PACKAGES="custom_msg_example"
# Terminal No.1, roscore
roscore
# Terminal No.2, publisher
cd /path/to/cpp_cookbook/3rdparty/ros
source devel/setup.bash
rosrun custom_msg_example publisher 
# Terminal No.3, subscriber
cd /path/to/cpp_cookbook/3rdparty/ros
source devel/setup.bash
rosrun custom_msg_example subscriber 
```
