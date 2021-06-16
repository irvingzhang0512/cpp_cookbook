#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include <opencv2/opencv.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);

  cv::VideoCapture cap(
      "/ssd01/zhangyiyang/rabbit_cpp_deployment/tests/data/passage_20s.mp4");
  assert(cap.isOpened());
  cv::Mat frame;
  bool flag;
  sensor_msgs::ImagePtr msg;
  ros::Rate loop_rate(10);
  while (nh.ok()) {
    flag = cap.read(frame);
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}