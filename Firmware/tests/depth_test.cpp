#include "CaptureDevice.h"
#include "Calibration.h"
#include "ProcessData.h"

int main(int argc, char **argv)
{
  /**Stereo camera device implementation*/
  CaptureDevice camDev;
  Calibration calib;

  std::string filename = "./DataProcessing/stereoCalibration.yml";

  cv::Mat imgL_raw, imgR_raw, imgL, imgR;

  if (argc > 1)
    filename = argv[1];

  calib.loadStereoCalibration(filename);

  camDev.Init();
  
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
    calib.rectifyStereo(imgL_raw, imgR_raw, imgL, imgR);

    // Process data
    auto [keypoints1, descriptors1] = single_ORB(imgL);
    auto [keypoints2, descriptors2] = single_ORB(imgR);

    auto matches = descriptor_matcher(descriptors1, descriptors2, 0.6);
    // auto points =  stereo_3Dpoints(P1, P2, keypoints1, keypoints2, matches);
    auto landmarks = stereo_landmarks(calib, keypoints1, keypoints2, descriptors1, descriptors2, matches);

    // Show data
    int k = 1;
    draw_and_show(imgL, imgR, k);
    for (const auto &lm : landmarks)
    {
      std::cout << lm.position << std::endl;
    }
    cv::waitKey(0);
  }

  return 0;
}