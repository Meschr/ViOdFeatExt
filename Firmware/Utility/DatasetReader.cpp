#include "DatasetReader.h"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <regex> // For regex-based splitting
#include <stdexcept>

namespace fs = std::filesystem;

void DatasetReader::loadImagePairs(const std::string &subfolderName,
                                   bool debugPrint) {
  // Construct the full path to the specified subfolder
  fs::path subfolderPath = datasetDirectory / subfolderName;

  // Ensure the subfolder exists
  if (!fs::exists(subfolderPath) || !fs::is_directory(subfolderPath)) {
    throw std::runtime_error(
        "Subfolder does not exist or is not a directory: " +
        subfolderPath.string());
  }

  std::cout << "Loading image pairs from: " << subfolderPath << std::endl;

  std::vector<std::string> leftImages, rightImages;

  // Iterate through files in the subfolder
  for (const auto &entry : fs::directory_iterator(subfolderPath)) {
    if (!entry.is_regular_file() || entry.path().extension() != ".png") {
      continue; // Skip non-regular files or files with unsupported formats
    }

    const std::string fileName = entry.path().filename().string();
    const std::string filePath = entry.path().string();

    // Filter files based on naming convention (e.g., "left" and "right")
    if (fileName.find("left") != std::string::npos) {
      leftImages.push_back(filePath);
      if (debugPrint)
        std::cout << "Left image found: " << filePath << std::endl;
    } else if (fileName.find("right") != std::string::npos) {
      rightImages.push_back(filePath);
      if (debugPrint)
        std::cout << "Right image found: " << filePath << std::endl;
    }
  }

  // Sort and ensure matching pairs
  std::sort(leftImages.begin(), leftImages.end());
  std::sort(rightImages.begin(), rightImages.end());

  if (leftImages.size() != rightImages.size()) {
    throw std::runtime_error(
        "Mismatch in left and right image counts in subfolder: " +
        subfolderPath.string());
  }

  // Add paired images to the queue
  for (size_t i = 0; i < leftImages.size(); ++i) {
    imagePairsQueue.push({leftImages[i], rightImages[i]});
  }

  std::cout << "Loaded " << leftImages.size() << " stereo pairs into the queue."
            << std::endl;
}

bool DatasetReader::nextStereoImagePair(cv::Mat &leftImage,
                                        cv::Mat &rightImage,
                                        float resolution) {
  if (imagePairsQueue.empty()) {
    return false; // No more images in the queue
  }

  // Get the next pair from the queue
  const auto [leftPath, rightPath] = imagePairsQueue.front();
  imagePairsQueue.pop();

  // load the images in grayscale

  leftImage = cv::imread(leftPath, cv::IMREAD_GRAYSCALE); 
  rightImage = cv::imread(rightPath, cv::IMREAD_GRAYSCALE);


  //in case we want to implement the kp tracking on binary images
  //cv::threshold(leftImage, leftImage, 100, 255, cv::THRESH_BINARY);
  //cv::threshold(rightImage, rightImage, 100, 255, cv::THRESH_BINARY);

  cv::resize(leftImage, leftImage, cv::Size(), resolution, resolution, cv::INTER_LINEAR);
  cv::resize(rightImage, rightImage, cv::Size(), resolution, resolution, cv::INTER_LINEAR);


  // Check if images loaded successfully
  if (leftImage.empty() || rightImage.empty()) {
    throw std::runtime_error("Failed to load images: " + leftPath + " or " +
                             rightPath);
  }

  std::cout << "Loaded stereo pair: " << leftPath << " and " << rightPath
            << std::endl;
  return true;
}

std::vector<cv::Point3f>
DatasetReader::parseTrajectoryData(const std::string &fileName) {
  std::vector<cv::Point3f> trajectory;

  // Build the complete file path using the dataset directory and file name
  std::filesystem::path filePath = datasetDirectory / fileName;

  // Open the file
  std::ifstream file(filePath);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open file: " + filePath.string());
  }

  std::string line;
  std::getline(file, line); // Read the header line and discard it

  int lineNumber = 2; // Start from line 2 (data starts after the header)
  while (std::getline(file, line)) {
    std::istringstream lineStream(line);
    std::string token;
    std::vector<std::string> tokens;

    // Tokenize the line by whitespace
    while (lineStream >> token) {
      tokens.push_back(token);
    }

    // Validate the number of columns in the line
    if (tokens.size() != 13) {
      std::cerr << "Warning: Skipping malformed line #" << lineNumber
                << " (expected 13 columns, found " << tokens.size()
                << "): " << line << std::endl;
      ++lineNumber;
      continue;
    }

    try {
      // Extract Robot_X, Robot_Y, Robot_Z for trajectory visualization
      float robotX = std::stof(tokens[4]); // Column 4: Robot_X
      float robotY = std::stof(tokens[5]); // Column 5: Robot_Y
      float robotZ = std::stof(tokens[6]); // Column 6: Robot_Z

      // Add the 3D point to the trajectory
      trajectory.emplace_back(robotX, robotY, robotZ);
    } catch (const std::exception &e) {
      std::cerr << "Error: Failed to parse numeric values in line #"
                << lineNumber << ": " << e.what() << std::endl;
    }

    ++lineNumber;
  }

  std::cout << "Loaded " << trajectory.size()
            << " points from file: " << fileName << std::endl;
  return trajectory;
}
