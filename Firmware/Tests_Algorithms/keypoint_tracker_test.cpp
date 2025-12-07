#include "CaptureDevice.h"
#include "Calibration.h"
#include "ProcessData.h"
#include "DatasetReader.h"
#include <iostream>

int main(int argc, char** argv) {
  DatasetReader reader;
  CaptureDevice camDev;
  Calibration calib;
  std::string filename = "../DataProcessing/stereoCalibration.yml";
  calib.loadStereoCalibration(filename);

  if (argc > 1) 
    filename = argv[1];

  try {
    //camDev.Init();
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }

  std::vector<Landmark> Previous_landmark_L ;

  const std::string subfolderName = "2025-11-18_11-59-45_linearMovementPointedIntoTheRoom400mm";
  reader.loadImagePairs(subfolderName, false);
  cv::Mat imgL_raw, imgR_raw, imgL, imgR, lastR, lastL;
  int frame_counter = 0;

  //-------------------------------------------------k --> 1 = ORB, 2 = BRISK------------------------------------------------------------------------------
  int k = 2; 

  while (reader.nextStereoImagePair(imgL_raw, imgR_raw)) {
    frame_counter++;
  

    calib.rectifyStereo(imgL_raw, imgR_raw, imgL, imgR);
    //cv::imshow("Rectified Left Image", imgL);
    //cv::imshow("Rectified Right Image", imgR);

    if (frame_counter == 0)
    {
      lastR = imgR.clone();
      lastL = imgL.clone();
      continue;
    }
    
    

    auto matches = img_to_matches(calib, imgL, imgR, k);
    auto landmarks = img_to_landmark(calib, imgL, imgR, k);
    lastR = imgR.clone();
    lastL = imgL.clone();

    //draw_landmark_kyp(imgL, landmarks);
    draw_and_show(imgL, imgR, k);

    std::cout << "Number of matches in frame: " << matches.size() << std::endl;
    frame_counter++;
    std::cout << "Frame: " << frame_counter << std::endl;
    
    
    if (cv::waitKey(0) == 'q') {
      break;
}}

  return 0;
}
