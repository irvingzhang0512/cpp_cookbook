# x264

## Overview

+ Features: Encode each frame from a local video to h264 bytes.

## Tutorial

```shell
# install x264
git clone https://code.videolan.org/videolan/x264.git
cd x264
./configure --enable-shared --disable-asm
make
sudo make install
cd ..

# test
mkdir build && cd build && cmake .. && make -j
./test_x264
```
