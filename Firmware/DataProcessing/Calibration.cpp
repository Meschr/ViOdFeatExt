#include "Calibration.h"
#include <vector>
#include <math.h>

void checkBoardCalibration(const cv::Size boardSize, 
                           const double squareSize, 
                           const std::vector<std::string>& leftFiles, 
                           const std::vector<std::string>& rightFiles, 
                           cv::Mat& K1, 
                           cv::Mat& D1, 
                           cv::Mat& K2, 
                           cv::Mat& D2, 
                           int flags) {
    // --- storage for calibration ---
    std::vector<std::vector<cv::Point3f>> objectPoints; 
    std::vector<std::vector<cv::Point2f>> imagePointsLeft, imagePointsRight;
    std::vector<cv::Point3f> obj;

    if (leftFiles.size() != rightFiles.size()){
        throw std::runtime_error("Error: Not the same number of files for right and left strings.");
        return;
    }

    for (int r = 0; r < boardSize.height; ++r) {
        for (int c = 0; c < boardSize.width; ++c) {
            obj.emplace_back(c * squareSize, r * squareSize, 0.0);
        }
    }

    cv::Size imageSize;
    for (size_t i = 0; i < leftFiles.size(); ++i) {
        // std::cout << leftFiles[i] << ", " << rightFiles[i] << "\n";
        cv::Mat left = cv::imread(leftFiles[i], cv::IMREAD_GRAYSCALE);
        cv::Mat right = cv::imread(rightFiles[i], cv::IMREAD_GRAYSCALE);
        
        if (left.empty() || right.empty()) {
            std::cerr << "Failed to load pair " << i << std::endl;
            continue;
        }
        if (imageSize == cv::Size()) 
            imageSize = left.size();

        std::vector<cv::Point2f> cornersL, cornersR;
        bool foundL = cv::findChessboardCorners(left, boardSize, cornersL,
                                                cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
        bool foundR = cv::findChessboardCorners(right, boardSize, cornersR,
                                                cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);


        if (!foundL || !foundR) {
            std::cout << "Chessboard not found in pair " << i << " (foundL=" << foundL << ", foundR=" << foundR << ")\n";
            continue;
        }
        
        // refine to subpixel accuracy
        cv::TermCriteria term(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 1e-6);
        cv::cornerSubPix(left, cornersL, cv::Size(11,11), cv::Size(-1,-1), term);
        cv::cornerSubPix(right, cornersR, cv::Size(11,11), cv::Size(-1,-1), term);

        // store
        imagePointsLeft.push_back(cornersL);
        imagePointsRight.push_back(cornersR);
        objectPoints.push_back(obj);

        std::cout << "Accepted pair " << i << " with " << cornersL.size() << " corners.\n";
    }

    if (objectPoints.size() < 5) {
        std::cerr << "Not enough valid pairs for stereoCalibrate. Need >= ~5 good pairs.\n";
        return;
    }

    // --- stereo calibration ---
    cv::Mat R, T, E, F;

    cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-6);

    double rms = cv::stereoCalibrate(objectPoints, imagePointsLeft, imagePointsRight,
                                     K1, D1, K2, D2, imageSize,
                                     R, T, E, F,
                                     flags, criteria);

    std::cout << "Stereo calibration RMS error = " << rms << std::endl;
    std::cout << "K1:\n" << K1 << "\nD1:\n" << D1 << "\nK2:\n" << K2 << "\nD2:\n" << D2 << std::endl;

    // // baseline in meters (since squareSize was in meters)
    // double baseline = cv::norm(T);
    // std::cout << "Baseline (meters): " << baseline << std::endl;

    // // --- rectify and get projection matrices and Q ---
    // cv::Mat R1, R2, Q;
    // cv::stereoRectify(K1, D1, K2, D2, imageSize, R, T, R1, R2, P1, P2, Q);

    // std::cout << "P1:\n" << P1 << "\nP2:\n" << P2 << "\nQ:\n" << Q << std::endl;

}

void saveStereoCalibration(const std::string& filename,
                           const cv::Size& imageSize,
                           const cv::Mat& K1, 
                           const cv::Mat& D1,
                           const cv::Mat& P1,
                           const cv::Mat& K2, 
                           const cv::Mat& D2,
                           const cv::Mat& P2,
                           const cv::Mat& R,  
                           const cv::Mat& T) {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        std::cerr << "Could not open file " << filename << " for writing\n";
        return;
    }

    fs << "image_width"  << imageSize.width;
    fs << "image_height" << imageSize.height;

    fs << "K1" << K1;
    fs << "D1" << D1;
    fs << "P1" << P1;
    fs << "K2" << K2;
    fs << "D2" << D2;
    fs << "P2" << P2;
    fs << "R"  << R;
    fs << "T"  << T;

    fs.release();
}

void intrinsicAproximation(const double focalLength, 
                           const double horizontalFoV, 
                           const double verticalFoV, 
                           const cv::Size& imageSize,
                           cv::Mat& K1, 
                           cv::Mat& D1,
                           cv::Mat& K2, 
                           cv::Mat& D2) {
 
    double sensor_w_mm = 2.0 * focalLength * tan(M_PI * horizontalFoV / 360.0);
    double sensor_h_mm = 2.0 * focalLength * tan(M_PI * verticalFoV  / 360.0);

    double fx = focalLength * (double(imageSize.width)  / sensor_w_mm);
    double fy = focalLength * (double(imageSize.height) / sensor_h_mm);

    double cx = imageSize.width/2;
    double cy = imageSize.height/2;

    cv::Mat K = (cv::Mat_<double>(3,3) <<
        fx, 0.0, cx,
        0.0, fy, cy,
        0.0, 0.0, 1.0);
    cv::Mat D = cv::Mat::zeros(1, 5, CV_64F); 

    K1 = K;
    K2 = K;

    D1 = D;
    D2 = D;
}
void extrinsicApproximation(const float baseline,
                            const cv::Mat& K1, 
                            const cv::Mat& K2, 
                            cv::Mat& P1, 
                            cv::Mat& P2,
                            cv::Mat& R,
                            cv::Mat& T) {
    cv::Mat Rt1; 
    cv::Mat Rt2; 

    // Transformation from left camera to right camera
    R = cv::Mat::eye(3,3, CV_64F);
    T = (cv::Mat_<double>(3,1) << baseline, 0.0, 0.0); 

    cv::hconcat(cv::Mat::eye(3,3,CV_64F), cv::Mat::zeros(3,1,CV_64F), Rt1);
    cv::hconcat(R, T, Rt2);

    P1 = K1 * Rt1;
    P2 = K2 * Rt2;
    
    return;
}   