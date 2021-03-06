#ifndef _RABBIT_H264_DECODER_H_
#define _RABBIT_H264_DECODER_H_

#include <string.h>

#include <opencv2/opencv.hpp>
// C++引用C语言的头文件
extern "C" {
#include "libavcodec/avcodec.h"
#include "libavfilter/avfilter.h"
#include "libavfilter/buffersink.h"
#include "libavfilter/buffersrc.h"
#include "libavformat/avformat.h"
#include "libavutil/avstring.h"
#include "libavutil/channel_layout.h"
#include "libavutil/dict.h"
#include "libavutil/fifo.h"
#include "libavutil/imgutils.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mathematics.h"
#include "libavutil/opt.h"
#include "libavutil/parseutils.h"
#include "libavutil/pixdesc.h"
#include "libavutil/samplefmt.h"
#include "libswresample/swresample.h"
}

class CH264Decoder {
 public:
  CH264Decoder();
  ~CH264Decoder();
  /*************************************************
    Function:initial
    Description:初始化
    Input:无
    Output:无
    Return:错误代码
    Others:无
  *************************************************/
  int initial();
  /*************************************************
    Function:decode
    Description:解码
    Input:pDataIn-待解码数据，nInSize-待解码数据长度
    Output:pDataOut-解码后的数据，nWidth-解码后的图像宽度，nHeight-解码后的图像高度
    Return:错误代码
    Others:解码后的数据为RGB16格式
  *************************************************/
  int decode(uint8_t *pDataIn, int nInSize, cv::Mat &res);
  /*************************************************
    Function:unInitial
    Description:销毁
    Input:无
    Output:无
    Return:无
    Others:无
  *************************************************/
  void unInitial();

 private:
  int avframe_to_cvmat(AVFrame *frame, cv::Mat &res);
  AVFrame *cvmat2avframe(cv::Mat mat);

 private:
  AVCodec *codec;
  AVCodecContext *context;
  AVFrame *frame;
  AVPacket packet;
};

#endif  // _RABBIT_H264_DECODER_H_
