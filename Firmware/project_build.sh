#!/bin/bash 

mkdir -p ./build
cd ./build

cmake -D CMAKE_C_COMPILER=gcc-13 \
      -D CMAKE_CXX_COMPILER=g++-13 \
      -D OpenCV_DIR=/usr/local/lib/cmake/opencv4 ..

cmake --build .
