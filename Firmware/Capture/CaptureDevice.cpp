#include "CaptureDevice.h"

void CaptureDevice::Init() {
  // Initialize left camera
  mLeftCam = new cv::VideoCapture(0);

  if (!mLeftCam->isOpened()) {
    std::cerr << "Error: Could not open left camera." << std::endl;
    return;
  }

  mRightCam = new cv::VideoCapture(1);
  if (!mRightCam->isOpened()) {
    std::cerr << "Error: Could not open right camera." << std::endl;
    return;
  }
  // TODO: set camera parameters and calibration

  // Wait for the camera to adjust
  cv::Mat frame;
  for (int i = 0; i < 5; ++i)
    *mLeftCam >> frame;
}

cv::Mat CaptureDevice::GetImage() {
  // Capture one frame
  cv::Mat frame;
  *mLeftCam >> frame;

  if (frame.empty()) {
    std::cerr << "Error: Captured empty frame." << std::endl;
    return cv::Mat();
  }

  return frame;
  // Display it
  // cv::imshow("Captured Image", frame);
  // cv::waitKey(0);
}

double CaptureDevice::GetFpsLeftCamera() {
  double fps = mLeftCam->get(cv::CAP_PROP_FPS);
  std::cout << "Left Camera FPS: " << fps << std::endl;
}

double CaptureDevice::GetFpsRightCamera() {
  double fps = mRightCam->get(cv::CAP_PROP_FPS);
  std::cout << "Right Camera FPS: " << fps << std::endl;
}
