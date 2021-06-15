#include "ros/ros.h"

int main(int argc,char **argv) 
{
  // name of this node is "hello_node"
  ros::init(argc,argv,"hello_node");
  ros::NodeHandle nh;
  // print string with roscpp function ORS_INFO_STREAM
  ROS_INFO_STREAM("hello world!!!");
}