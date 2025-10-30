#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>

void TestGetImage();

class CaptureDevice {
public:
  void Init();
  cv::Mat GetImage();

private:
  cv::VideoCapture *mLeftCam;
  cv::VideoCapture *mRightCam;
};
