# OpenCV cookbook

## Basic usage

+ [codes](./src/opencv_cookbook.cc)

## freetype example

+ 在图片中写入中文，需要先安装 freetype。
+ 如何安装 freetype？
  + 从[这里](https://download.savannah.gnu.org/releases/freetype/)下载源码。
  + 源码编译就是普通的cmake编译 `mkdir build && cd build && cmake .. && make && sudo make install`
+ 还需要准备字体文件，这里网上随便下载了一个作为测试。
+ [codes](./src/opencv_freetype_cookbook.cc)
