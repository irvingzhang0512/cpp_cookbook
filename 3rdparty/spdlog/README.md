# Spdlog Cookbook

## Overview

+ Sptdlog is a widely used 

## Install

+ C++ 11 compiler is nedded

```shell
git clone https://github.com/gabime/spdlog.git
cd spdlog && mkdir build && cd build
cmake .. && make -j
```

## Cookbook

+ For logging utils, the following features are needed
    + Create difference sinks for the same logger.
    + Set logger level(global/sink).
    + Set logging format.
