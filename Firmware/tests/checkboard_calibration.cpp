#include "Calibration.h"

#include <dirent.h>
#include <vector>
#include <fstream>
#include <string>
#include <iostream>

std::vector<std::string> readCSVColumn(const std::string& dir, const std::string& filename, std::string headerName, int readEvery = 1) {
    std::vector<std::string> column;

    std::ifstream file(dir + filename);
    if (!file.is_open()) {
      std::cerr << "Failed to open file: " << dir + filename << std::endl;
      return column;
    }

    int colIndex = 0;
    std::string line;
    if (!std::getline(file, line)) {  // read the header row
      std::cerr << "Failed to get header row" << std::endl;
      return column;
    }else{
      std::string cell;
      std::stringstream ss(line);
      while (std::getline(ss, cell, ',')) {
        if (cell == headerName) {
          break;
        }
        ++colIndex;
      }
    }

    int csvRow = 0;
    while (std::getline(file, line)) {
        int idx = 0;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) {
            if ((idx == colIndex) and (csvRow%readEvery == 0)) {
                column.push_back(dir + cell);
                break;
            }
            ++idx;
        }
        csvRow++;
    }

    return column;
}

int main(int argc, char** argv) {
    if (argc == 0) {
      std::cerr << "Usage: " << argv[0] << " <directory_path>\n";
      return 1;
    }

    std::string dir = argv[1];

    std::vector<std::string> leftFiles  = readCSVColumn(dir, "data.csv", "left_image" , 1);
    std::vector<std::string> rightFiles = readCSVColumn(dir, "data.csv", "right_image", 1);

    if (0 == leftFiles.size() || 0== rightFiles.size())
      return 1;

    std::cout << "Left images: "  << std::to_string(leftFiles.size())  << "\n";
    std::cout << "Right images: " << std::to_string(rightFiles.size()) << "\n";

    
    Calibration calib;
    calib.checkBoardCalibration(cv::Size(10, 7), 0.025, leftFiles, rightFiles);

    calib.saveStereoCalibration(argv[2]);

    return 0;
}


