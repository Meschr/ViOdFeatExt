#include <iostream>
#include <opencv2/opencv.hpp>

void checkBoardCalibration(const cv::Size boardSize, 
                           const double squareSize, 
                           const std::vector<std::string>& leftFiles, 
                           const std::vector<std::string>& rightFiles, 
                           cv::Mat& K1, 
                           cv::Mat& D1, 
                           cv::Mat& K2, 
                           cv::Mat& D2, 
                           int flags=0);

void saveStereoCalibration(const std::string& filename,
                           const cv::Size& imageSize,
                           const cv::Mat& K1, 
                           const cv::Mat& D1,
                           const cv::Mat& P1,
                           const cv::Mat& K2, 
                           const cv::Mat& D2,
                           const cv::Mat& P2,
                           const cv::Mat& R,  
                           const cv::Mat& T);

void intrinsicAproximation(const double focalLength, 
                           const double horizontalFoV, 
                           const double verticalFoV, 
                           const cv::Size& imageSize,
                           cv::Mat& K1, 
                           cv::Mat& D1,
                           cv::Mat& K2, 
                           cv::Mat& D2);

void extrinsicApproximation(const float baseline,
                            const cv::Mat& K1, 
                            const cv::Mat& K2, 
                            cv::Mat& P1, 
                            cv::Mat& P2,
                            cv::Mat& R,
                            cv::Mat& T);                           