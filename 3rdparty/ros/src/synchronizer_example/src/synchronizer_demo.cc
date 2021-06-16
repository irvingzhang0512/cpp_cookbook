#include <custom_msg_example/can_data.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>

void callback(const sensor_msgs::ImageConstPtr& image,
              const custom_msg_example::can_dataConstPtr& info) {
  printf("Info\n");
  printf("Header/\n");
  printf("  seq %d\n", (*info).header.seq);
  printf("  frame_id %s\n", (*info).header.frame_id.c_str());
  printf("id %d\n", (*info).id);
  printf("len %d\n", (*info).len);
  printf("data [%d,%d,%d,%d]\n\n", (*info).data[0], (*info).data[1],
         (*info).data[2], (*info).data[3]);
  cv::Mat frame = cv_bridge::toCvShare(image, "bgr8")->image;
  printf("Frame\n");
  printf("%d, %d\n", frame.rows, frame.cols);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "message_filter_node");
  ros::Time::init();
  ros::NodeHandle nh;

  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "camera/image",
                                                            1);
  message_filters::Subscriber<custom_msg_example::can_data> info_sub(
      nh, "can_data", 1);
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, custom_msg_example::can_data>
      MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), image_sub,
                                                   info_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  ros::spin();
  return 0;
}
