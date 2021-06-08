#include <iostream>

#include "x264_encoder.h"

int main() {
  std::cout << "Hello World!" << std::endl;
  cv::Mat frame, frame_dst;
  cv::VideoCapture cap("../../../data/yolo_test.mp4");
  assert(cap.isOpened());
  x264Encoder m_x264Encoder((int)cap.get(cv::CAP_PROP_FRAME_WIDTH),
                            (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT), 3,
                            (int)cap.get(cv::CAP_PROP_FPS));
  int jpeg_quality = 75;
  std::vector<int> params;
  params.push_back(cv::IMWRITE_JPEG_QUALITY);
  params.push_back(jpeg_quality);

  while (1) {
    cap >> frame;
    if (frame.empty()) break;
    int size = m_x264Encoder.EncodeOneFrame(frame);
    uchar* data = nullptr;
    data = m_x264Encoder.GetEncodedFrame();
    std::cout << size << std::endl;
  }

  cap.release();
  return 0;
}