#!/bin/bash 

cd ~/Software/ViOdFeatExt/Firmware/build/

cmake -DOpenCV_DIR=/usr/local/lib/cmake/opencv4 ..

cmake --build .
