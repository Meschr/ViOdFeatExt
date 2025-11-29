#include "CaptureDevice.h"
#include "Calibration.h"
#include "ProcessData.h"

int main(int argc, char **argv)
{
  /**Stereo camera device implementation*/
  CaptureDevice camDev;

  std::string filename = "stereoCalibration.yml";
  cv::Size imageSize = cv::Size(640, 480);
  cv::Mat K1, K2;
  cv::Mat D1, D2;
  cv::Mat P1, P2;
  cv::Mat R1, R2;
  cv::Mat R, T, Q;

  cv::Mat imgL_raw, imgR_raw, imgL, imgR;

  if (argc > 1)
    filename = argv[1];

  loadStereoCalibration(filename, imageSize, K1, D1, P1, K2, D2, P2, R, T, Q);

  cv::Rect validRoi1, validRoi2;
  cv::stereoRectify(
      K1, D1,
      K2, D2,
      imageSize,
      R, // rotation from left to right
      T, // translation from left to right
      R1, R2,
      P1, P2,
      Q,
      cv::CALIB_ZERO_DISPARITY, // makes principal points align vertically
      0,                        // alpha (0 = crop, 1 = keep all)
      imageSize,
      &validRoi1, &validRoi2);

  cv::Mat map11, map12, map21, map22;

  cv::initUndistortRectifyMap(
      K1, D1, R1, P1, imageSize, CV_16SC2, map11, map12);

  cv::initUndistortRectifyMap(
      K2, D2, R2, P2, imageSize, CV_16SC2, map21, map22);

  try
  {
    camDev.Init();
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << '\n';
    return 1;
  }

  for (;;)
  {
    // Get data
    try
    {
      imgL_raw = camDev.GetLeftImage();
      imgR_raw = camDev.GetRightImage();
    }
    catch (const std::exception &e)
    {
      std::cout << e.what() << std::endl;
      continue;
    }

    cv::Mat imgL, imgR;
    cv::remap(imgL_raw, imgL, map11, map12, cv::INTER_LINEAR);
    cv::remap(imgR_raw, imgR, map21, map22, cv::INTER_LINEAR);

    // Process data
    auto [keypoints1, descriptors1] = single_ORB(imgL);
    auto [keypoints2, descriptors2] = single_ORB(imgR);

    auto matches = descriptor_matcher(descriptors1, descriptors2, 0.6);
    // auto points =  stereo_3Dpoints(P1, P2, keypoints1, keypoints2, matches);
    auto landmarks = stereo_landmarks(P1, P2, keypoints1, keypoints2, descriptors1, descriptors2, matches);

    // Show data
    draw_and_show(imgL, imgR, keypoints1, keypoints2, matches);
    for (const auto &lm : landmarks)
    {
      std::cout << lm.position << std::endl;
    }
    cv::waitKey(0);
  }

  return 0;
}