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
      "2025-11-18_11-59-45_linearMovementPointedIntoTheRoom400mm";

  reader.loadImagePairs(subfolderName, false);

  auto [timestamps, trajectory] = reader.parseTrajectoryData(
      "RoboticArmMovements_251118/LOG251118_121811.txt");

  visualizeTrajectory(trajectory);

  cv::Mat imgL_raw, imgR_raw, imgL, imgR, lastL, lastR;
  std::pair<std::vector<cv::KeyPoint>, cv::Mat> lastFeaturesL, lastFeaturesR;
  std::vector<cv::DMatch> lastMatches;
  std::vector<Landmark> allLandmarks;
  int k = 1;
  int frame_counter = 0;
  while (reader.nextStereoImagePair(imgL_raw, imgR_raw, 1.0)) {
    frame_counter++;
    calib.rectifyStereo(imgL_raw, imgR_raw, imgL, imgR);

    // Process data
    auto [keypointsL, descriptorsL] = single_ORB(imgL);
    auto [keypointsR, descriptorsR] = single_ORB(imgR);

    auto matches = descriptor_matcher(descriptorsL, descriptorsR, 0.6);
    std::cout << "Frame " << frame_counter << ": " << matches.size()
              << " matches found between left and right image." << std::endl;
    draw_and_show(imgL, imgR, keypointsL, keypointsR, matches);

    if (frame_counter == 1) {
      lastFeaturesL = {keypointsL, descriptorsL};
      lastFeaturesR = {keypointsR, descriptorsR};
      lastMatches = matches;
      lastR = imgR.clone();
      lastL = imgL.clone();

      continue;
    }

    // Now match with last frame to get motion
    auto motionMatches =
        descriptor_matcher(descriptorsL, lastFeaturesL.second, 0.6);
    std::cout << "Frame " << frame_counter << ": " << motionMatches.size()
              << " matches found between current and last left image."
              << std::endl;

    draw_and_show(imgL, lastL, keypointsL, lastFeaturesL.first, motionMatches);

    // Compute landmarks
    auto landmarks =
        stereo_landmarks(calib, keypointsL, keypointsR, descriptorsL,
                         descriptorsR, lastMatches, motionMatches);
    allLandmarks.insert(allLandmarks.end(), landmarks.begin(), landmarks.end());
    std::cout << "Frame " << frame_counter << ": " << landmarks.size()
              << " 3D landmarks computed." << std::endl;

    lastFeaturesL = {keypointsL, descriptorsL};
    lastFeaturesR = {keypointsR, descriptorsR};
    lastMatches = matches;
    lastR = imgR.clone();
    lastL = imgL.clone();

    /*
    std::vector<cv::Point3f> positions;
    for (const auto &lm : landmarks) {
      std::cout << lm.position << std::endl;
      positions.push_back(lm.position);
    }

    visualize3DKeypoints(positions);*/
    // Wait for a key press to proceed (for debugging/visualization)
    if (cv::waitKey(0) == 'q') {
      break; // Exit on 'q'
    }
  }

  return 0;
}
