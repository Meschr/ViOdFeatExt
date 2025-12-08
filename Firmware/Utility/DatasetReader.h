#pragma once

#include <filesystem> // For C++17 or higher
#include <iostream>
#include <opencv2/opencv.hpp>
#include <queue>
#include <string>

namespace fs = std::filesystem;

class DatasetReader {
public:
  explicit DatasetReader(const std::string &datasetPath = "../../Datasets/")
      : datasetDirectory(fs::absolute(datasetPath)), currentIndex(0) {
    std::cout << "DatasetReader initialized with path: " << datasetDirectory
              << std::endl;
  }

  // Function to load stereo image pairs from a specific subfolder
  void loadImagePairs(const std::string &subfolderName,
                      bool debugPrint = false);

  // Function to retrieve the next stereo image pair
  bool nextStereoImagePair(cv::Mat &leftImage, cv::Mat &rightImage, float resolution);

  // Function to parse trajectory data from the log file of the robotic arm
  std::pair<std::vector<double>, std::vector<cv::Point3f>> parseTrajectoryData(const std::string &fileName);

private:
  fs::path datasetDirectory;
  std::queue<std::pair<std::string, std::string>>
      imagePairsQueue; // Queue of stereo pairs
  size_t currentIndex;
};
