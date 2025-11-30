#include "Calibration.h"
#include "DatasetReader.h"
#include "ProcessData.h"
#include "VisualizationHelper.h"
#include <iostream>
#include <vector>

int main() {
  DatasetReader reader;
  Calibration calib;
  std::string filename = "../DataProcessing/stereoCalibration.yml";
  calib.loadStereoCalibration(filename);

  // 2025-11-18_11-47-30_linearMovement1TowardsSatellite500mm
  // 2025-11-18_11-59-45_linearMovementPointedIntoTheRoom400mm
  // 2025-11-18_12-08-38_linearMovementPointedTowardsMoon300mm
  // 2025-11-18_12-18-21_ 2AxisMovementPointedTowardsMoon250mmSquare
  // 2025-11-18_12-27-10_angularMovementRoomMoon90Deg

  const std::string subfolderName =
      "2025-11-18_11-47-30_linearMovement1TowardsSatellite500mm";

  reader.loadImagePairs(subfolderName, false);

  auto trajectory = reader.parseTrajectoryData(
      "RoboticArmMovements_251118/LOG251118_121811.txt");

  visualizeTrajectory(trajectory);

  cv::Mat imgL_raw, imgR_raw, imgL, imgR;

  while (reader.nextStereoImagePair(imgL_raw, imgR_raw)) {
    calib.rectifyStereo(imgL_raw, imgR_raw, imgL, imgR);
    cv::imshow("Rectified Left Image", imgL);
    cv::imshow("Rectified Right Image", imgR);

    // Process data
    auto [keypoints1, descriptors1] = single_ORB(imgL);
    auto [keypoints2, descriptors2] = single_ORB(imgR);

    auto matches = descriptor_matcher(descriptors1, descriptors2, 0.6);
    auto landmarks = stereo_landmarks(calib, keypoints1, keypoints2,
                                      descriptors1, descriptors2, matches);

    std::vector<cv::Point3f> positions;
    for (const auto &lm : landmarks) {
      std::cout << lm.position << std::endl;
      positions.push_back(lm.position);
    }
    visualize3DKeypoints(positions);
    // Wait for a key press to proceed (for debugging/visualization)
    if (cv::waitKey(0) == 'q') {
      break; // Exit on 'q'
    }
  }

  return 0;
}
