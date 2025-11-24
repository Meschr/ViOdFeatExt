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

    loadStereoCalibration(filename, imageSize, K1, D1, P1, K2, D2, P2, R, T);

    std::cout << "K1:\n" << K1 << "\n"
              << "D1:\n" << D1 << "\n"
              << "P1:\n" << P1 << "\n"
              << "K2:\n" << K2 << "\n"
              << "D2:\n" << D2 << "\n"
              << "P2:\n" << P2 << "\n"
              << "R: \n" << R  << "\n"
              << "T: \n" << T  << "\n"
              << std::endl;
    // TODO: Implement depth stimation
    return 0;
}