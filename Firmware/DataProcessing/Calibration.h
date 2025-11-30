#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>

class Calibration
{
public:
  cv::Mat K1, D1, K2, D2, R1, R2, P1, P2, Q, R, T;
  cv::Mat map11, map12, map21, map22;
  cv::Size imageSize;

  void checkBoardCalibration(const cv::Size boardSize,
                             const double squareSize,
                             const std::vector<std::string> &leftFiles,
                             const std::vector<std::string> &rightFiles,
                             int flags = 0);

  void saveStereoCalibration(const std::string &filename);

  void loadStereoCalibration(const std::string &filename);

  void intrinsicAproximation(const double focalLength,
                             const double horizontalFoV,
                             const double verticalFoV,
                             const cv::Size &imageSize,
                             cv::Mat &K1,
                             cv::Mat &D1,
                             cv::Mat &K2,
                             cv::Mat &D2);

  void extrinsicApproximation(const float baseline,
                              const cv::Mat &K1,
                              const cv::Mat &K2,
                              cv::Mat &P1,
                              cv::Mat &P2,
                              cv::Mat &R,
                              cv::Mat &T);

  void rectifyStereo(const cv::Mat &imgL_raw,
                     const cv::Mat &imgR_raw,
                     cv::Mat &imgL,
                     cv::Mat &imgR);

private:
  void calcIntrisicExtrinsicParameters(void);
  void calcRectifyParameters(void);
  void calcRectifyMap(void);
  std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f>> getChessboardPoints(const std::vector<std::string> &leftFiles,
                                                                                                 const std::vector<std::string> &rightFiles,
                                                                                                 const int index,
                                                                                                 const cv::Size boardSize);
};