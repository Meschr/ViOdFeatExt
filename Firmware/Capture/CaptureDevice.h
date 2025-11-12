#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>

void TestGetImage();

struct DataPacket {
  int frameNumber;
  int64_t timestamp;
  cv::Mat leftImage;
  cv::Mat rightImage;
  float accX;
  float accY;
  float accZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  float magX;
  float magY;
  float magZ;
};

class CaptureDevice {
public:
  void Init();
  void InitForWebCam();
  cv::Mat GetLeftImage();
  cv::Mat GetRightImage();
  cv::Mat GetStereoImage();
  double GetFpsLeftCamera();
  double GetFpsRightCamera();

private:
  cv::VideoCapture mLeftCam;
  cv::VideoCapture mRightCam;
};
