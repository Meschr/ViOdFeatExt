#include "CaptureDevice.h"

void CaptureDevice::Init() {
  // Initialize left camera
  mLeftCam = cv::VideoCapture(
      "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=640, "
      "height=480, format=(string)NV12, framerate=(fraction)20/1 ! nvvidconv "
      "flip-method=0 ! video/x-raw, width=640, height=480, format=(string)BGRx "
      "! videoconvert ! video/x-raw, format=(string)BGR ! appsink",
      cv::CAP_GSTREAMER);

  if (!mLeftCam.isOpened()) {
    throw std::runtime_error("Error: Could not open left camera.");
    return;
  }

  mRightCam = cv::VideoCapture(
      "nvarguscamerasrc sensor-id=1 ! video/x-raw(memory:NVMM), width=640, "
      "height=480, format=(string)NV12, framerate=(fraction)20/1 ! nvvidconv "
      "flip-method=0 ! video/x-raw, width=640, height=480, format=(string)BGRx "
      "! videoconvert ! video/x-raw, format=(string)BGR ! appsink",
      cv::CAP_GSTREAMER);
  if (!mRightCam.isOpened()) {
    throw std::runtime_error("Error: Could not open right camera.");
    return;
  }
  // TODO: set camera parameters and calibration

  // Wait for the camera to adjust
  cv::Mat frame;
  for (int i = 0; i < 5; ++i)
    mLeftCam >> frame;
}

void CaptureDevice::InitForWebCam() {
  // Initialize left camera
  mLeftCam = cv::VideoCapture(0);

  if (!mLeftCam.isOpened()) {
    throw std::runtime_error("Error: Could not open webcam.");
    return;
  }

  // Wait for the camera to adjust
  cv::Mat frame;
  for (int i = 0; i < 5; ++i)
    mLeftCam >> frame;
}

cv::Mat CaptureDevice::GetLeftImage() {
  // Capture one frame
  cv::Mat frame;
  mLeftCam >> frame;

  if (frame.empty()) {
    throw std::runtime_error("Error: Captured empty frame.");
    return cv::Mat();
  }

  return frame;
}

cv::Mat CaptureDevice::GetRightImage() {
  // Capture one frame
  cv::Mat frame;
  mRightCam >> frame;

  if (frame.empty()) {
    throw std::runtime_error("Error: Captured empty frame.");
    return cv::Mat();
  }

  return frame;
}

cv::Mat CaptureDevice::GetStereoImage() {
  // Capture one frame
  cv::Mat frame;

  cv::hconcat(CaptureDevice::GetLeftImage(), CaptureDevice::GetRightImage(),
              frame);

  if (frame.empty()) {
    throw std::runtime_error("Error: Captured empty frame.");
    return cv::Mat();
  }

  return frame;
}

double CaptureDevice::GetFpsLeftCamera() {
  double fps = mLeftCam.get(cv::CAP_PROP_FPS);
  std::cout << "Left Camera FPS: " << fps << std::endl;
  return fps;
}

double CaptureDevice::GetFpsRightCamera() {
  double fps = mRightCam.get(cv::CAP_PROP_FPS);
  std::cout << "Right Camera FPS: " << fps << std::endl;
  return fps;
}
