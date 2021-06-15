#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv) {
  // create node with node name
  ros::init(argc, argv, "publish_node");
  ros::NodeHandle nh;

  // define topic "chatter" and related types `String` with buffer size 1000
  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

  // limit the rate of loop
  ros::Rate loop_rate(10);

  int count = 0;

  while (ros::ok()) {
    // ros::ok() is designed for Ctrl+C/ros::shutdown/...

    // prepare msg for publisher
    std_msgs::String msg;
    std::stringstream ss;
    ss << "how are you " << count;
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());

    // publish msg
    chatter_pub.publish(msg);

    // call function
    ros::spinOnce();

    // limit the rate of loop
    loop_rate.sleep();

    ++count;
  }

  return 0;
}