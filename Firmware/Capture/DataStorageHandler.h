#pragma once

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>

class DataStorageHandler {
public:
  DataStorageHandler(const std::string &baseDir = "../../LogData/")
      : baseDirectory(baseDir) {}
  bool Init();
  std::string SaveImage(const cv::Mat &img, const std::string &prefix);
  void SaveData(double accX, double accY, double accZ, double gyroX,
                double gyroY, double gyroZ, double magX, double magY,
                double magZ, const std::string &leftImageName,
                const std::string &rightImageName);

private:
  std::filesystem::path baseDirectory;
  std::string subfolderName;
  std::filesystem::path outputDirectory;
  std::filesystem::path csvFilePath;
  std::ofstream csvFile;
  int frameCounter = 0;
};
