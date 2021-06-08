# ffmpeg

## Overview

## Tutorial

```shell
# install
sudo apt install libx264-dev
sudo apt install ffmpeg libav-tools libavcodec-dev libavdevice-dev libavfilter-dev libavformat-dev libavresample-dev libavutil-dev libpostproc-dev libswresample-dev libswscale-dev

# build
rm -rf build && mkdir build && cd build && cmake .. && make -j

# test
./ffmpeg_h264
```

## Cookbook

+ h264 encoder and decoder sample
