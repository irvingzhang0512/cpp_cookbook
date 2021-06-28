#include <opencv2/freetype.hpp>
#include <opencv2/opencv.hpp>
#include <string>

int main(int argc, char **argv) {
  std::string text = "你好，世界！";
  cv::Ptr<cv::freetype::FreeType2> ft2;
  ft2 = cv::freetype::createFreeType2();
  ft2->loadFontData("../../../data/SourceHanSerifCN-Regular-1.otf", 0);
  auto img = cv::imread("../../../data/zidane.jpg");

  int font_height = 30; // height of text
  int thickness = -1;
  int linestyle = 8;
  int baseline = 0;
  cv::Size textSize = ft2->getTextSize(text, font_height, thickness, &baseline);
  if (thickness > 0) {
    baseline += thickness;
  }
  cv::Point textOrg(
      (img.cols - textSize.width) / 2,
      (img.rows + textSize.height) / 2);  // bottom left point in (w, h) order
  ft2->putText(img, text, textOrg, font_height, cv::Scalar::all(255), thickness,
               linestyle, true);
  cv::imshow("demo", img);
  cv::waitKey(0);

  return 0;
}