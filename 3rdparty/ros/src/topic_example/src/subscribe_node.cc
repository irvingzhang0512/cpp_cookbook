#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char** argv) {
  // create node with node name
  ros::init(argc, argv, "subscribe_node");
  ros::NodeHandle nh;

  // After receive a msg from "chatter", callback function will be called
  ros::Subscriber chatter_sub = nh.subscribe("chatter", 1000, chatterCallback);

  // hang thread, similar to `join`
  ros::spin();

  return 0;
}