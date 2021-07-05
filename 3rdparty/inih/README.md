# Inih

## Overview

+ Load cfg files.

## Install

+ There are two headers and two source codes.

## Cookbook

+ Cookbook could be found [here](./src/main.cc)

```cpp
#include <iostream>

#include "INIReader.h"

int main() {
  INIReader reader("../cfg/test.cfg");

  if (reader.ParseError() < 0) {
    std::cout << "Can't load 'test.ini'\n";
    return 1;
  }

  std::cout << "Config loaded from 'test.ini': version="
            << reader.GetInteger("protocol", "version", -1)
            << ", name=" << reader.Get("user", "name", "UNKNOWN")
            << ", email=" << reader.Get("user", "email", "UNKNOWN")
            << ", pi=" << reader.GetReal("user", "pi", -1)
            << ", active=" << reader.GetBoolean("user", "active", true) << "\n";
  return 0;
}
```
