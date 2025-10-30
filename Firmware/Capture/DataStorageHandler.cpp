#include "DataStorageHandler.h"

bool DataStorageHandler::Init() {
  std::cout << "Enter subfolder name: ";
  std::getline(std::cin, subfolderName);
  if (subfolderName.empty()) {
    std::cerr << "Error: Subfolder name cannot be empty.\n";
    return false;
  }

  outputDirectory = baseDirectory / subfolderName;
  if (!std::filesystem::exists(outputDirectory)) {
    std::filesystem::create_directories(outputDirectory);
  }
  // open CSV file
  csvFilePath = outputDirectory / "data.csv";
  csvFile.open(csvFilePath, std::ios::out);
  if (!csvFile.is_open()) {
    std::cerr << "Error: Could not open CSV file for writing.\n";
    return false;
  }

  // write header
  csvFile << "timestamp,acc_x,acc_y,acc_z,"
          << "gyro_x,gyro_y,gyro_z,"
          << "mag_x,mag_y,mag_z,"
          << "left_image,right_image\n";
  return true;
}

std::string DataStorageHandler::SaveImage(const cv::Mat &img,
                                          const std::string &prefix) {
  if (img.empty()) {
    std::cerr << "Error: Empty image provided to SaveImage.\n";
    return "";
  }

  std::ostringstream filename;
  filename << prefix << "_" << std::setw(6) << std::setfill('0')
           << frameCounter++ << ".png";
  std::string path = (outputDirectory / filename.str()).string();
  cv::imwrite(path, img);
  return filename.str();
}

void DataStorageHandler::SaveData(double accX, double accY, double accZ,
                                  double gyroX, double gyroY, double gyroZ,
                                  double magX, double magY, double magZ,
                                  const std::string &leftImageName,
                                  const std::string &rightImageName) {
  auto now = std::chrono::system_clock::now();
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch())
                .count();

  csvFile << ms << "," << accX << "," << accY << "," << accZ << "," << gyroX
          << "," << gyroY << "," << gyroZ << "," << magX << "," << magY << ","
          << magZ << "," << leftImageName << "," << rightImageName << "\n";
}
