# ROS cookbook

## Preview

+ Need to install ros first.
+ Example List
  + [Hello World Example](#example-no-1---hello-world-example): What you need to do to create a ROS app.
  + [Publisher and Subscribe](#publisher-and-subscriber): String demo for publisher and subsciber.
  + [Create Custom Message](#create-custom-message): Create a custom message, use publisher/subscriber to send/receive messages.
  + [OpenCV Image Example](#opencv-image-example): Create a publisher and subscriber for image data.
  + [Filter Demo](#filter-demo): Create a filter to load data from mutiple topic and fuse as one.

## Example No. 1 - Hello World Example

+ source codes [here](./src/hello_world)
+ Pileline
  + Step 1: Create workspace
  + Step 2: Create package
  + Step 3: Add related codes. Modify `package.xml` & `CMakeLists.txt`, add `src/hello_world.cc`
  + Step 4: Run `hello_world` Node.

+ Step 1: Create workspace

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

+ Step 2: Create package

```shell
cd /path/to/cpp_cookbook/3rdparty/ros

cd src
# the name of this pkg is `hello_world`, create a folder in `src/hello_world`
# and two files `CMakeLists.txt` and `package.xml` are created
catkin_create_pkg hello_world
```

+ Step 3: Add Hello World code in pkg
  + Modify `src/hello_world/src/hello_world.cc`
  + Modify pkg cmake config, aka `src/hello_world/CMakeLists.txt` and `src/hello_world/package.xml`
  + Modify `src/hello_world/package.xml`

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

+ Step 4: Compile and run
  + Compile
    + Compile the whole workspace: `cd /path/to/cpp_cookbook/3rdparty/ros && catkin_make`
    + Compile single pkg: `cd /path/to/cpp_cookbook/3rdparty/ros && catkin_make -DCATKIN_WHITELIST_PACKAGES="hello_world"`
  + Run:

```shell
# Terminal No.1
source devel/setup.bash
roscore

# Terminal No.2
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
source devel/setup.bash
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

# 3. Create msg file `can_data.msg` in `src/custom_msg_example/msg`, compile to generate header file.
#   Please note that, we have to generate header file first, or `#include <custom_msg_example/can_data.h>` will raise header file not found error.
catkin_make -DCATKIN_WHITELIST_PACKAGES="custom_msg_example"

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
source devel/setup.bash
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

## OpenCV Image Example

+ Create a publisher/subscriber to send/receive images.
+ If using another version of OpenCV(instead of OpenCV in ROS), cv_bridge should be modified.
  + Method 1: Build cv_bridge from source.
  + Method 2: Modify OpenCV related configs in cv_bridge, related [notes](https://blog.csdn.net/weixin_43436587/article/details/107711866)

```shell
cd /path/to/cpp_cookbook/3rdparty/ros

# 1. Make sure workspace is probably created.
# 2. Create pkg with dependencies `roscpp` and `std_msgs`
cd src
catkin_create_pkg image_opencv_example roscpp cv_bridge image_transport
cd ..

# 3. Modify configs: add opencv related configs in `CMakeLists.txt`, include `find_package(OpenCV)` and `target_link_libraries(target_name ${OpenCV_LIBS})`

# 4. Add image_publisher.cc and image_subscriber.cc
# 5. compile and run
catkin_make -DCATKIN_WHITELIST_PACKAGES="image_opencv_example"
# Terminal No.1, roscore
source devel/setup.bash
roscore
# Terminal No.2, publisher
cd /path/to/cpp_cookbook/3rdparty/ros
source devel/setup.bash
rosrun image_opencv_example image_publisher 
# Terminal No.3, subscriber
cd /path/to/cpp_cookbook/3rdparty/ros
source devel/setup.bash
rosrun image_opencv_example image_subscriber 
```

## Filter Demo

+ Use Time Synchronizer to read data from multiple topics and fuse data from these topics.
  + Read data from previous pkgs, custom msg publisher and image publisher.

```shell

cd /path/to/cpp_cookbook/3rdparty/ros

# 1. Make sure workspace is probably created.
# 2. Create pkg with dependencies
cd src
catkin_create_pkg synchronizer_example roscpp cv_bridge image_transport std_msgs custom_msg_example message_filters
cd ..

# 3. Modify configs: add opencv related configs in `CMakeLists.txt`, include `find_package(OpenCV)` and `target_link_libraries(target_name ${OpenCV_LIBS})`

# 4. Add synchronizer_demo.cc
# 5. compile and run(need to run two publisher from previous pkgs)
catkin_make -DCATKIN_WHITELIST_PACKAGES="synchronizer_example"
# Terminal No.1, roscore
source devel/setup.bash
roscore
# Terminal No.2, image publisher
cd /path/to/cpp_cookbook/3rdparty/ros
source devel/setup.bash
rosrun image_opencv_example image_publisher 
# Terminal No.3, custom msg publisher
cd /path/to/cpp_cookbook/3rdparty/ros
source devel/setup.bash
rosrun custom_msg_example publisher 
# Terminal No.4, synchronizer demo
cd /path/to/cpp_cookbook/3rdparty/ros
source devel/setup.bash
rosrun synchronizer_example synchronizer_demo 
```
