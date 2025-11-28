#include "Calibration.h"

int main(int argc, char** argv) {
    std::string filename = "stereoCalibration.yml";
    cv::Size imageSize = cv::Size(640, 480);
    cv::Mat K1; cv::Mat K2;
    cv::Mat D1; cv::Mat D2;
    cv::Mat P1; cv::Mat P2;
    cv::Mat R;
    cv::Mat T;

    if (argc > 1) 
        filename = argv[1];

    intrinsicAproximation(2.6, 73.0, 50.0, imageSize, K1, D1, K2, D2);
    extrinsicApproximation(0.06, K1, K2, P1, P2, R, T);
    

    std::cout << "K1:\n" << K1 << "\nD1:\n" << D1 << "\nK2:\n" << K2 << "\nD2:\n" << D2 << std::endl;

    saveStereoCalibration(filename, imageSize, K1, D1, P1, K2, D2, P2, R, T);
    
    return 0;
}