#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>

void TestGetImage();

class CaptureDevice {
public:
  void Init();
  cv::Mat GetLeftImage();
  cv::Mat GetRightImage();
  cv::Mat GetStereoImage();
  double GetFpsLeftCamera();
  double GetFpsRightCamera();

private:
  cv::VideoCapture mLeftCam;
  cv::VideoCapture mRightCam;
};
