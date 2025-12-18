#include "CaptureDevice.h"
#include "Calibration.h"
#include "ProcessData.h"
#include "DatasetReader.h"
#include <iostream>
#include <fstream>
#include "Transformations.h"

int main(int argc, char **argv)
{

  DatasetReader reader;
  Calibration calib;
  std::string filename = "../DataProcessing/stereoCalibration.yml";
  calib.loadStereoCalibration(filename);

  const std::string subfolderName = "2025-11-18_12-18-21_ 2AxisMovementPointedTowardsMoon250mmSquare";
  
  std::ofstream csvFile("../tests/translations.csv", std::ios::trunc);

  reader.loadImagePairs(subfolderName, false);

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

  cv::Mat imgL_raw, imgR_raw, imgL, imgR, lastR, lastL;
  int frame_counter = 0;

  //--------------------------------------------------- resolution = 0.5 means half the size of the image----------------------------------------
  float resolution = 1.;

  while (reader.nextStereoImagePair(imgL_raw, imgR_raw, resolution))
  {

    frame_counter++;
    // Skip frames that are not every 5th
    if (frame_counter % 5 != 0)
      continue;

    calib.rectifyStereo(imgL_raw, imgR_raw, imgL, imgR);
    auto landmarks = img_to_landmark(calib, imgL, imgR, BRISK_DESCRIPTOR);

    std::cout << "Frame: " << frame_counter << std::endl;

    cv::Affine3d transformation;

    if (!last_landmarks.empty() && transformation_calculation(landmarks, last_landmarks, transformation, RANSAC_SVD_CERES))
    {
      // std::cout << "Estimated Transformation:\n" << transformation.translation() << std::endl;
      cv::Vec3d translation = transformation.translation();
      csvFile << translation[0] << "," << translation[1] << "," << translation[2] << "\n";
    }else{
      csvFile << "0, 0, 0\n";
    }

    last_landmarks.clear();
    last_landmarks.reserve(landmarks.size());
    for (auto &lm : landmarks) {
        Landmark copy = lm;
        copy.descriptor = lm.descriptor.clone();
        last_landmarks.push_back(copy);
    }
  }
  csvFile.close();
  return 0;
}
