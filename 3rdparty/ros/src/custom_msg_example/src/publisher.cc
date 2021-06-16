#include "ros/ros.h"
#include "custom_msg_example/can_data.h"


int main(int argc, char **argv)
{
  printf("start\n");
  ros::init(argc, argv, "custom_msg_talker");
  ros::NodeHandle nh;
  printf("init ros\n");

  ros::Publisher pub_can_data = nh.advertise<custom_msg_example::can_data>("can_data", 10);
  printf("init publisher\n");
  ros::Rate loop_rate(10);
  int count = 0;

  while (ros::ok())
  {  
    printf("start loop\n");
    custom_msg_example::can_data output;
    
    output.header.seq = count;
    output.header.stamp = ros::Time::now();
    output.header.frame_id = "can_data";
    output.id = 0x101;
    output.len = 2;
    output.data.push_back(10);
    output.data.push_back(20);
    output.data.push_back(30);
    output.data.push_back(40);
    printf("create msg\n");
    
    pub_can_data.publish(output);
    printf("publish msg\n");
	
    ros::spinOnce();
    loop_rate.sleep();
    count++;
  }


  return 0;
}