#include "custom_msg_example/can_data.h"
#include "ros/ros.h"

void customMsgCallback(const custom_msg_example::can_data msg) {
  printf("Header/\n");
  printf("  seq %d\n", msg.header.seq);
  printf("  frame_id %s\n", msg.header.frame_id.c_str());
  printf("id %d\n", msg.id);
  printf("len %d\n", msg.len);
  printf("data [%d,%d,%d,%d]\n", msg.data[0], msg.data[1], msg.data[2],
         msg.data[3]);
  printf(msg.data.length());
//   printf("data [%d,%d,%d,%d]\n", msg.data.at(0), msg.data.at(1), msg.data.at(2),
//          msg.data.at(3));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "custom_msg_listener");
  ros::NodeHandle nh;
  ros::Subscriber sub_string = nh.subscribe("can_data", 10, customMsgCallback);
  ros::spin();
  return 0;
}
