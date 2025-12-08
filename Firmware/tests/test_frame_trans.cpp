#include "CaptureDevice.h"
#include "Calibration.h"
#include "ProcessData.h"
#include "DatasetReader.h"
#include <iostream>

int main(int argc, char **argv)
{
  DatasetReader reader("../../LogData/");
  CaptureDevice camDev;
  Calibration calib;
  std::string filename = "../DataProcessing/stereoCalibration.yml";
  calib.loadStereoCalibration(filename);

  if (argc > 1)
    filename = argv[1];

  try
  {
    // camDev.Init();
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << '\n';
    return 1;
  }

  std::vector<Landmark> last_landmarks;

  const std::string subfolderName = "2025-11-18_11-59-45_linearMovementPointedIntoTheRoom400mm";
  reader.loadImagePairs(subfolderName, false);
  cv::Mat imgL_raw, imgR_raw, imgL, imgR, lastR, lastL;
  int frame_counter = 0;

  //-------------------------------------------------k --> 1 = ORB, 2 = BRISK------------------------------------------------------------------------------
  int k = 1;
  //--------------------------------------------------- resolution = 0.5 means half the size of the image----------------------------------------
  float resolution = 1.;

  while (reader.nextStereoImagePair(imgL_raw, imgR_raw, resolution))
  {
    calib.rectifyStereo(imgL_raw, imgR_raw, imgL, imgR);

    auto landmarks = img_to_landmark(calib, imgL, imgR, k);

    std::cout << "Frame: " << frame_counter << std::endl;

    if (!last_landmarks.empty())
    {
      cv::Mat descriptors_current, descriptors_last;
      cv::Mat positions_current, positions_last;

      for (const auto &lm : landmarks)
      {
        descriptors_current.push_back(lm.descriptor);
        positions_current.push_back(lm.position);
      }

      for (const auto &lm : last_landmarks)
      {
        descriptors_last.push_back(lm.descriptor);
        positions_last.push_back(lm.position);
      }

      auto matches = descriptor_matcher(descriptors_current, descriptors_last, 0.7f);
      std::cout << "Number of matches with last frame: " << matches.size() << std::endl;

      positions_current.reserve(matches.size());
      positions_last.reserve(matches.size());

      for (const auto &m : matches)
      {
        positions_last.push_back(last_landmarks[m.queryIdx].position);
        positions_current.push_back(landmarks[m.trainIdx].position);
      }

      frame_counter++;
      last_landmarks = landmarks;
    }

    return 0;
  }
}