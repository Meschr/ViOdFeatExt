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

  cv::Mat imgL1, imgR1, imgL2, imgR2, imgL3, imgR3;

  if (argc > 1) 
    filename = argv[1];

  loadStereoCalibration(filename, imageSize, K1, D1, P1, K2, D2, P2, R, T);
  
  try {
    //camDev.Init();
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }

    // gotta add the relative path!!!!!!!!!
    //Frame 1
    std::string image_path1 = "/home/robinazv/ViOdFeatExt/LogData/2025-11-18_1/2025-11-18_11-47-30_linearMovement1TowardsSatellite500mm/left_001316.png";
    std::string image_path2 = "/home/robinazv/ViOdFeatExt/LogData/2025-11-18_1/2025-11-18_11-47-30_linearMovement1TowardsSatellite500mm/right_001317.png";
    
    //Frame 2
    std::string image_path3 = "/home/robinazv/ViOdFeatExt/LogData/2025-11-18_1/2025-11-18_11-47-30_linearMovement1TowardsSatellite500mm/left_002318.png";
    std::string image_path4 = "/home/robinazv/ViOdFeatExt/LogData/2025-11-18_1/2025-11-18_11-47-30_linearMovement1TowardsSatellite500mm/right_002319.png";
    
    //Frame 3
    std::string image_path5 = "/home/robinazv/ViOdFeatExt/LogData/2025-11-18_1/2025-11-18_11-47-30_linearMovement1TowardsSatellite500mm/left_002320.png";
    std::string image_path6 = "/home/robinazv/ViOdFeatExt/LogData/2025-11-18_1/2025-11-18_11-47-30_linearMovement1TowardsSatellite500mm/right_002321.png";
    
    imgL1 = cv::imread(image_path1, cv::IMREAD_GRAYSCALE); //Left image frame 1
    imgL2 = cv::imread(image_path3, cv::IMREAD_GRAYSCALE); //Left image frame 2
    imgL3 = cv::imread(image_path5, cv::IMREAD_GRAYSCALE); //Left image frame 3

    auto landmarks = path_to_landmark(image_path1, image_path2, P1, P2);
    auto landmarks2 = path_to_landmark(image_path3, image_path4, P1, P2);
    auto landmarks3 = path_to_landmark(image_path5, image_path6, P1, P2);

    std::cout << "Number of landmarks in frame 1: " << landmarks.size() << std::endl;
    std::cout << "Number of landmarks in frame 2: " << landmarks2.size() << std::endl;
    std::cout << "Number of landmarks in frame 3: " << landmarks3.size() << std::endl;
   


    //fidn matches between frame 1 adn frame 2 landmarks based on landmarks descriptors
    cv::Mat descriptors_lm1, descriptors_lm2, descriptors_lm3;

    for (const auto &lm : landmarks) {
        descriptors_lm1.push_back(lm.descriptor);
    }
    for (const auto &lm : landmarks2) {
        descriptors_lm2.push_back(lm.descriptor);
    }
    for (const auto &lm : landmarks3) {
        descriptors_lm3.push_back(lm.descriptor);
    }

    auto matches_lm = descriptor_matcher(descriptors_lm1, descriptors_lm2, 0.7);
    std::cout << "Matches between landmarks of frame 1 and frame 2: " << matches_lm.size() << std::endl;

    auto matches_lm3 = descriptor_matcher(descriptors_lm2, descriptors_lm3, 0.7);
    std::cout << "Matches between landmarks of frame 2 and frame 3: " << matches_lm3.size() << std::endl;



    // assign the IDS for fram 2
    int current_frame = 2; // for frame 2
    int local_id = 1;       // counter for new landmarks in this frame

    std::vector<bool> matched(landmarks2.size(), false);

    for (const auto &m : matches_lm) {
        int idx1 = m.queryIdx;
        int idx2 = m.trainIdx;

        landmarks2[idx2].id = landmarks[idx1].id; // keep the matched ID from frame 1
        matched[idx2] = true;
    }

    for (size_t i = 0; i < landmarks2.size(); ++i) {
        if (!matched[i]) {
            landmarks2[i].id = std::to_string(current_frame) + "_" + std::to_string(local_id++);
        }
    }

    for (const auto &lm : landmarks2) {
        std::cout << "ID: " << lm.id << " 3D pos: " << lm.position
                  << " keypoint 2D: " << lm.keypoint.pt << std::endl;
    }


    



    // assign IDs for frame 3
    int current_frame3 = 3;
    int local_id3 = 1;
    std::vector<bool> matched3(landmarks3.size(), false);

    for (const auto &m : matches_lm3) {
        int idx2 = m.queryIdx;  // index in frame 2
        int idx3 = m.trainIdx;  // index in frame 3

        // matched → carry ID from frame 2
        landmarks3[idx3].id = landmarks2[idx2].id;
        matched3[idx3] = true;
    }

    for (size_t i = 0; i < landmarks3.size(); ++i) {
        if (!matched3[i]) {
            landmarks3[i].id = 
                std::to_string(current_frame3) + "_" + std::to_string(local_id3++);
        }
    }


    //show the images with the ids one after the other
    draw_landmark_kyp(imgL1, landmarks);
    cv::waitKey(0);
    draw_landmark_kyp(imgL2, landmarks2);
    cv::waitKey(0);
    draw_landmark_kyp(imgL3, landmarks3);
    cv::waitKey(0); 



    // susbtract the 3d point location between frome 1 and 2
    for (const auto &m : matches_lm) {
        int idx1 = m.queryIdx; // frame 1
        int idx2 = m.trainIdx; // frame 2

        cv::Point3f pos1 = landmarks[idx1].position;
        cv::Point3f pos2 = landmarks2[idx2].position;

        cv::Point3f diff = pos2 - pos1;

        std::cout << "Landmark ID: " << landmarks2[idx2].id 
                  << " Movement (X,Y,Z): (" << diff.x << ", " << diff.y << ", " << diff.z << ")\n";
    }

  return 0;
}
