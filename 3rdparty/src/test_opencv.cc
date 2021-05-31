#include <opencv2/opencv.hpp>

void image_operations() {
  cv::Mat img = cv::imread("../data/zidane.jpg");

  // cv::imshow("demo", img);
  // cv::waitKey(0);
  cv::imwrite("./test.jpg", img);

  // norm: (img/alpha) + beta
  cv::Mat processed_img;
  float alpha = 1 / 255.;
  float beta = 1;
  processed_img.convertTo(img, CV_32FC3, alpha, beta);

  // NHWC2NCHW + BGR2RGB
  float res[img.cols * img.rows * 3];
  cv::Mat tp;
  cv::Mat bgr[3];
  cv::split(tp, bgr);
  for (int i = 0; i < 3; i++) {
    int idx = i * tp.rows * tp.cols;
    int ch = 2 - i;
    memcpy((void *)&res[idx], (void *)bgr[ch].data,
           tp.rows * tp.cols * sizeof(float));
  }
}

void video_operations() {
  cv::VideoCapture cap("../data/yolo_test.mp4");
  // cv::VideoCapture cap;
  // cap.open("../data/yolo_test.mp4");
  assert(cap.isOpened());
  cap.get(cv::CAP_PROP_POS_FRAMES);  // next frame idx
  cap.get(cv::CAP_PROP_FRAME_WIDTH);
  cap.get(cv::CAP_PROP_FRAME_HEIGHT);
  cap.get(cv::CAP_PROP_FPS);
  cap.get(cv::CAP_PROP_FOURCC);
  cap.get(cv::CAP_PROP_FRAME_COUNT);

  cv::VideoWriter writer;
  // mp4 -> mp4v
  // avi -> XVID
  writer.open("./test.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
              cap.get(cv::CAP_PROP_FPS),
              cv::Size(cap.get(cv::CAP_PROP_FRAME_WIDTH),
                       cap.get(cv::CAP_PROP_FRAME_HEIGHT)));

  cv::Mat frame;
  bool flag;
  while (true) {
    flag = cap.read(frame);
    if (!flag) break;

    writer << frame;
  }

  writer.release();
  cap.release();
}

void mat_operations() {
  // initialize

  // Create an empty mat
  // The third arg is a macro
  // 8/16/32/64 is the bytes of a single element
  // U/S/F is the data type
  // C1/C2/C3/C4 is the number of channels
  cv::Mat m1(10, 5, CV_8UC3);

  // Create a mat with Scalar
  // Reference: https://blog.csdn.net/laoqiuge/article/details/36629209
  // [1, 2, 3, 1, 2, 3, ...]
  cv::Mat m21(10, 5, CV_8SC3, cv::Scalar(1, 2, 3));
  // [1, 2, 0, 1, 2, 0, ...]
  cv::Mat m22(10, 5, CV_8SC3, cv::Scalar(1, 2));
  // [1, 0, 0, 1, 0, 0, ...]
  cv::Mat m23(10, 5, CV_8SC3, cv::Scalar(1));
  // [1, 2, 3, 1, 2, 3, ...]
  cv::Mat m24(10, 5, CV_8SC3, cv::Scalar(1, 2, 3, 4));
  // [1, 1, 1, 1, 1, 1, ...]
  cv::Mat m25(10, 5, CV_8SC3, cv::Scalar::all(1));

  // Convert between cv::Mat and float*
  float src[20 * 20 * 3];
  for (int i = 0; i < 20 * 20 * 3; i++) src[i] = 0;
  cv::Mat m3(20, 20, CV_16FC3, src);
  float *data = (float *)m3.data;

  // Some useful initialize methods
  cv::Mat mz = cv::Mat::zeros(cv::Size(5, 5), CV_8UC3);  // 全零矩阵
  cv::Mat mo = cv::Mat::ones(cv::Size(5, 5), CV_8UC3);   // 全1矩阵
  cv::Mat me = cv::Mat::eye(cv::Size(5, 5), CV_32FC3);  // 对角线为1的对角矩阵
  cv::Mat mtranspose = mz.t();

  // random matrix
  // it seems that normal distribution only support 32F or 64F
  cv::Mat m(224, 224, CV_32FC3);
  cv::randn(m, 0., 1.);                                     // mean/std
  cv::randu(m, cv::Scalar::all(0.), cv::Scalar::all(255));  // low/hieght
  cv::RNG rng(0);                                           // random seed
  rng.fill(m, cv::RNG::UNIFORM, 0, 255, false);
  rng.fill(m, cv::RNG::NORMAL, 0, 1, false);
  float rnadom_number = (float)rng.next();

  // base calculation
  cv::Mat matrix1(5, 10, CV_32FC1);
  cv::Mat matrix2(5, 10, CV_32FC1);
  cv::Mat res(5, 10, CV_32FC1);
  rng.fill(matrix1, cv::RNG::UNIFORM, 1, 255, false);
  rng.fill(matrix2, cv::RNG::UNIFORM, 1, 255, false);
  // element wise operations
  cv::add(matrix1, matrix2, res);
  cv::subtract(matrix1, matrix2, res);
  cv::multiply(matrix1, matrix2, res);
  cv::divide(matrix1, matrix2, res);
  res = cv::abs(matrix1);
  res = matrix1.mul(matrix2);
  // matrix & scalar
  res = matrix1 * 0.5;
  res = matrix1 / 0.5;
  res = matrix1 - 0.5;
  res = matrix1 + 0.5;

  // matrix multiply
  auto r = matrix1 * matrix2.t();
}

int main() {
  image_operations();
  video_operations();
  mat_operations();
}