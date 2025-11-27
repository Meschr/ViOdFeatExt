#include "CaptureDevice.h"
#include "Calibration.h"
#include "ProcessData.h"

int main(int argc, char** argv) {
  /**Stereo camera device implementation*/
  CaptureDevice camDev;

  std::string filename = "stereoCalibration.yml";
  cv::Size imageSize = cv::Size(640, 480);
  cv::Mat K1, K2;
  cv::Mat D1, D2;
  cv::Mat P1, P2;
  cv::Mat R, T;

  cv::Mat imgL, imgR;

  if (argc > 1) 
    filename = argv[1];

  loadStereoCalibration(filename, imageSize, K1, D1, P1, K2, D2, P2, R, T);
  
  try {
    camDev.Init();
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }

  for (;;) {
    // Get data
    try {
      imgL = camDev.GetLeftImage();
      imgR = camDev.GetRightImage();
    } catch (const std::exception &e) {
      std::cout << e.what() << std::endl;
      continue;
    }

    // Process data
    auto [keypoints1, descriptors1] = single_ORB(imgL);
    auto [keypoints2, descriptors2] = single_ORB(imgR);

    auto matches = descriptor_matcher(descriptors1, descriptors2, 0.65);
    // auto points =  stereo_3Dpoints(P1, P2, keypoints1, keypoints2, matches);
    auto landmarks = stereo_landmarks(P1, P2, keypoints1, keypoints2, descriptors1, descriptors2, matches);


    // Show data
    draw_and_show(imgL, imgR, keypoints1, keypoints2, matches);
    for (const auto& lm : landmarks) {
      std::cout << lm.position.z << std::endl;
    }
    cv::waitKey(0);
  }
  

  return 0;
}