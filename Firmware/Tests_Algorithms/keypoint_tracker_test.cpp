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
  while (reader.nextStereoImagePair(imgL_raw, imgR_raw)) {
    frame_counter++;
  

    calib.rectifyStereo(imgL_raw, imgR_raw, imgL, imgR);
    cv::imshow("Rectified Left Image", imgL);
    cv::imshow("Rectified Right Image", imgR);

    if (frame_counter == 0)
    {
      lastR = imgR.clone();
      lastL = imgL.clone();
      continue;
    }
    
    

    //std::vector<cv::Point3f> points = img_to_3dpoints(calib, imgL, imgR); // L + R give in the image, get back 3d points and the corresponding vector with the IDs
    auto matches = img_to_matches(calib, imgL, lastL);
    
    lastR = imgR.clone();
    lastL = imgL.clone();
    //check how many matches found


    std::cout << "Number of matches in frame " << frame_counter << ": " << matches.size() << std::endl;
    //auto landmarks = img_to_landmark(calib, imgL, imgR);
    frame_counter++;
    std::cout << "Frame: " << frame_counter << std::endl;
    if (cv::waitKey(0) == 'q') {
      break;
}}
    // if frame nr is o then skip,else find matches between previous and current landmarks.descriptior


    // for (const auto &lm : landmarks) {
    //     descriptors_lm1.push_back(lm.descriptor);
    // }
    // for (const auto &lm : landmarks2) {
    //     descriptors_lm2.push_back(lm.descriptor);
    // }
    // for (const auto &lm : landmarks3) {
    //     descriptors_lm3.push_back(lm.descriptor);
    // }



    // if (frame_counter == 0) {
    //   Previous_landmark_L = landmarks;
    // } else {
    //   cv::Mat current_descriptor = landmarks.descriptor;
    //   cv::Mat previous_descriptor = Previous_landmark_L.descriptor;
    //   auto matches_lm = descriptor_matcher(previous_descriptor, current_descriptor, 0.7);
    //   std::cout << "Matches between landmarks of previous and current frame: " << matches_lm.size() << std::endl;
    //   cv::waitKey(0);
    // }
      


    // auto landmarks = path_to_landmark(image_path1, image_path2);
    // auto landmarks2 = path_to_landmark(image_path3, image_path4);
    // auto landmarks3 = path_to_landmark(image_path5, image_path6);

    // std::cout << "Number of landmarks in frame 1: " << landmarks.size() << std::endl;
    // std::cout << "Number of landmarks in frame 2: " << landmarks2.size() << std::endl;
    // std::cout << "Number of landmarks in frame 3: " << landmarks3.size() << std::endl;
   


    //fidn matches between frame 1 adn frame 2 landmarks based on landmarks descriptors
    // cv::Mat descriptors_lm1, descriptors_lm2, descriptors_lm3;

    // for (const auto &lm : landmarks) {
    //     descriptors_lm1.push_back(lm.descriptor);
    // }
    // for (const auto &lm : landmarks2) {
    //     descriptors_lm2.push_back(lm.descriptor);
    // }
    // for (const auto &lm : landmarks3) {
    //     descriptors_lm3.push_back(lm.descriptor);
    // }

    // auto matches_lm = descriptor_matcher(descriptors_lm1, descriptors_lm2, 0.7);
    // std::cout << "Matches between landmarks of frame 1 and frame 2: " << matches_lm.size() << std::endl;

    // auto matches_lm3 = descriptor_matcher(descriptors_lm2, descriptors_lm3, 0.7);
    // std::cout << "Matches between landmarks of frame 2 and frame 3: " << matches_lm3.size() << std::endl;



    // // assign the IDS for fram 2
    // int current_frame = 2; // for frame 2
    // int local_id = 1;       // counter for new landmarks in this frame

    // std::vector<bool> matched(landmarks2.size(), false);

    // for (const auto &m : matches_lm) {
    //     int idx1 = m.queryIdx;
    //     int idx2 = m.trainIdx;

    //     landmarks2[idx2].id = landmarks[idx1].id; // keep the matched ID from frame 1
    //     matched[idx2] = true;
    // }

    // for (size_t i = 0; i < landmarks2.size(); ++i) {
    //     if (!matched[i]) {
    //         landmarks2[i].id = std::to_string(current_frame) + "_" + std::to_string(local_id++);
    //     }
    // }

    // for (const auto &lm : landmarks2) {
    //     std::cout << "ID: " << lm.id << " 3D pos: " << lm.position
    //               << " keypoint 2D: " << lm.keypoint.pt << std::endl;
    // }


    



    // // assign IDs for frame 3
    // int current_frame3 = 3;
    // int local_id3 = 1;
    // std::vector<bool> matched3(landmarks3.size(), false);

    // for (const auto &m : matches_lm3) {
    //     int idx2 = m.queryIdx;  // index in frame 2
    //     int idx3 = m.trainIdx;  // index in frame 3

    //     // matched → carry ID from frame 2
    //     landmarks3[idx3].id = landmarks2[idx2].id;
    //     matched3[idx3] = true;
    // }

    // for (size_t i = 0; i < landmarks3.size(); ++i) {
    //     if (!matched3[i]) {
    //         landmarks3[i].id = 
    //             std::to_string(current_frame3) + "_" + std::to_string(local_id3++);
    //     }
    // }


    // //show the images with the ids one after the other
    // draw_landmark_kyp(imgL1, landmarks);
    // cv::waitKey(0);
    // draw_landmark_kyp(imgL2, landmarks2);
    // cv::waitKey(0);
    // draw_landmark_kyp(imgL3, landmarks3);
    // cv::waitKey(0); 



    // // susbtract the 3d point location between frome 1 and 2
    // for (const auto &m : matches_lm) {
    //     int idx1 = m.queryIdx; // frame 1
    //     int idx2 = m.trainIdx; // frame 2

    //     cv::Point3f pos1 = landmarks[idx1].position;
    //     cv::Point3f pos2 = landmarks2[idx2].position;

    //     cv::Point3f diff = pos2 - pos1;

    //     std::cout << "Landmark ID: " << landmarks2[idx2].id 
    //               << " Movement (X,Y,Z): (" << diff.x << ", " << diff.y << ", " << diff.z << ")\n";
    // }

  return 0;
}
