#!/bin/bash 

cd ./build/

cmake -DOpenCV_DIR=/usr/local/lib/cmake/opencv4 ..

cmake --build .
