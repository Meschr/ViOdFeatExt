#include "Calibration.h"
#include <vector>
#include <math.h>

void Calibration::calcRectifyParameters(void)
{
  if (K1.empty() || D1.empty() || 
      K2.empty() || D2.empty() ||
      R.empty()  || T.empty())
      throw std::runtime_error("Stereo parameters not defined");

  cv::Rect validRoi1, validRoi2;
  cv::stereoRectify(
      K1, D1,
      K2, D2,
      imageSize,
      R, // rotation from left to right
      T, // translation from left to right
      R1, R2,
      P1, P2,
      Q,
      cv::CALIB_ZERO_DISPARITY, // makes principal points align vertically
      0,                        // alpha (0 = crop, 1 = keep all)
      imageSize,
      &validRoi1, &validRoi2);
}

std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f>> Calibration::getChessboardPoints(const std::vector<std::string> &leftFiles,
                                                                                               const std::vector<std::string> &rightFiles,
                                                                                               const int index,
                                                                                               const cv::Size boardSize)
{
  cv::Mat left = cv::imread(leftFiles[index], cv::IMREAD_GRAYSCALE);
  cv::Mat right = cv::imread(rightFiles[index], cv::IMREAD_GRAYSCALE);

  std::vector<cv::Point2f> cornersL, cornersR;

  if (left.empty() || right.empty())
  {
    std::cerr << "Failed to load pair " << index << std::endl;
    return {cornersL, cornersR};
  }
  if (imageSize == cv::Size())
    imageSize = left.size();

  bool foundL = cv::findChessboardCorners(left, boardSize, cornersL,
                                          cv::CALIB_CB_ADAPTIVE_THRESH |
                                              cv::CALIB_CB_NORMALIZE_IMAGE |
                                              cv::CALIB_CB_FAST_CHECK);
  bool foundR = cv::findChessboardCorners(right, boardSize, cornersR,
                                          cv::CALIB_CB_ADAPTIVE_THRESH |
                                              cv::CALIB_CB_NORMALIZE_IMAGE |
                                              cv::CALIB_CB_FAST_CHECK);

  if (!foundL || !foundR)
  {
    std::vector<cv::Point2f> cornersL, cornersR;
    std::cout << "Chessboard not found in pair " << index << " (foundL=" << foundL << ", foundR=" << foundR << ")\n";
    return {cornersL, cornersR};
  }

  // refine to subpixel accuracy
  cv::TermCriteria term(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 1e-6);
  cv::cornerSubPix(left, cornersL, cv::Size(11, 11), cv::Size(-1, -1), term);
  cv::cornerSubPix(right, cornersR, cv::Size(11, 11), cv::Size(-1, -1), term);

  std::cout << "Accepted pair " << index << " with " << cornersL.size() << " corners.\n";
  
  return {cornersL, cornersR} ;
}

void Calibration::checkBoardCalibration(const cv::Size boardSize,
                                        const double squareSize,
                                        const std::vector<std::string> &leftFiles,
                                        const std::vector<std::string> &rightFiles,
                                        int flags)
{
  // --- storage for calibration ---
  std::vector<std::vector<cv::Point3f>> objectPoints;
  std::vector<std::vector<cv::Point2f>> imagePointsLeft, imagePointsRight;
  std::vector<cv::Point3f> obj;

  if (leftFiles.size() != rightFiles.size())
  {
    throw std::runtime_error("Error: Not the same number of files for right and left strings.");
    return;
  }

  for (int r = 0; r < boardSize.height; ++r)
  {
    for (int c = 0; c < boardSize.width; ++c)
    {
      obj.emplace_back(c * squareSize, r * squareSize, 0.0);
    }
  }

  for (size_t i = 0; i < leftFiles.size(); ++i)
  {
    auto [cornersL, cornersR] = getChessboardPoints(leftFiles, rightFiles, i, boardSize);
    // store
    if (cornersL.empty() || cornersR.empty())
      continue;
      
    imagePointsLeft.push_back(cornersL);
    imagePointsRight.push_back(cornersR);
    objectPoints.push_back(obj); 
  }

  if (objectPoints.size() < 5)
  {
    std::cerr << "Not enough valid pairs for stereoCalibrate. Need >= ~5 good pairs.\n";
    return;
  }

  // --- stereo calibration ---
  cv::Mat E, F;

  cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-6);

  double rms = cv::stereoCalibrate(objectPoints, imagePointsLeft, imagePointsRight,
                                   K1, D1, K2, D2, imageSize,
                                   R, T, E, F,
                                   flags, criteria);


  double baseline = cv::norm(T);
  std::cout << "Baseline (meters): " << baseline << std::endl;

  calcRectifyParameters();
}

void Calibration::saveStereoCalibration(const std::string &filename)
{
  cv::FileStorage fs(filename, cv::FileStorage::WRITE);
  if (!fs.isOpened())
  {
    std::cerr << "Could not open file " << filename << " for writing\n";
    return;
  }

  fs << "image_width" << imageSize.width;
  fs << "image_height" << imageSize.height;

  fs << "K1" << K1;
  fs << "D1" << D1;
  fs << "K2" << K2;
  fs << "D2" << D2;
  fs << "R" << R;
  fs << "T" << T;

  fs.release();
}

void Calibration::loadStereoCalibration(const std::string &filename)
{
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  if (!fs.isOpened())
  {
    std::cerr << "Could not open file " << filename << " for reading\n";
    return;
  }

  int w, h;
  fs["image_width"] >> w;
  fs["image_height"] >> h;
  imageSize = cv::Size(w, h);

  fs["K1"] >> K1;
  fs["D1"] >> D1;
  fs["K2"] >> K2;
  fs["D2"] >> D2;
  fs["R"] >> R;
  fs["T"] >> T;

  fs.release();
}

void Calibration::intrinsicAproximation(const double focalLength,
                                        const double horizontalFoV,
                                        const double verticalFoV,
                                        const cv::Size &imageSize,
                                        cv::Mat &K1,
                                        cv::Mat &D1,
                                        cv::Mat &K2,
                                        cv::Mat &D2)
{

  double sensor_w_mm = 2.0 * focalLength * tan(M_PI * horizontalFoV / 360.0);
  double sensor_h_mm = 2.0 * focalLength * tan(M_PI * verticalFoV / 360.0);

  double fx = focalLength * (double(imageSize.width) / sensor_w_mm);
  double fy = focalLength * (double(imageSize.height) / sensor_h_mm);

  double cx = imageSize.width / 2;
  double cy = imageSize.height / 2;

  cv::Mat K = (cv::Mat_<double>(3, 3) << fx, 0.0, cx,
               0.0, fy, cy,
               0.0, 0.0, 1.0);
  cv::Mat D = cv::Mat::zeros(1, 5, CV_64F);

  K1 = K;
  K2 = K;

  D1 = D;
  D2 = D;
}
void extrinsicApproximation(const float baseline,
                            const cv::Mat &K1,
                            const cv::Mat &K2,
                            cv::Mat &P1,
                            cv::Mat &P2,
                            cv::Mat &R,
                            cv::Mat &T)
{
  cv::Mat Rt1;
  cv::Mat Rt2;

  // Transformation from left camera to right camera
  R = cv::Mat::eye(3, 3, CV_64F);
  T = (cv::Mat_<double>(3, 1) << -baseline, 0.0, 0.0);

  cv::hconcat(cv::Mat::eye(3, 3, CV_64F), cv::Mat::zeros(3, 1, CV_64F), Rt1);
  cv::hconcat(R, T, Rt2);

  P1 = K1 * Rt1;
  P2 = K2 * Rt2;

  return;
}

void Calibration::calcRectifyMap()
{
  if (K1.empty() || D1.empty() || R1.empty() || P1.empty() ||
      K2.empty() || D2.empty() || R2.empty() || P2.empty())
    calcRectifyParameters();

  cv::initUndistortRectifyMap(K1, D1, R1, P1, imageSize, CV_16SC2, map11, map12);
  cv::initUndistortRectifyMap(K2, D2, R2, P2, imageSize, CV_16SC2, map21, map22);
}

void Calibration::rectifyStereo(const cv::Mat &imgL_raw,
                                const cv::Mat &imgR_raw,
                                cv::Mat &imgL,
                                cv::Mat &imgR)
{
  if (map11.empty() || map12.empty() || map21.empty() || map22.empty())
    calcRectifyMap();

  cv::remap(imgL_raw, imgL, map11, map12, cv::INTER_LINEAR);
  cv::remap(imgR_raw, imgR, map21, map22, cv::INTER_LINEAR);
}
