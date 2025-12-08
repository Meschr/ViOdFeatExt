#pragma once

#include "Calibration.h"
#include <opencv2/opencv.hpp>

struct Landmark {
  cv::Point3f position;
  cv::Mat descriptor;
  cv::KeyPoint keypoint;
};

std::vector<cv::KeyPoint> single_FAST(const cv::Mat &img);
std::pair<std::vector<cv::KeyPoint>, cv::Mat> single_ORB(const cv::Mat &img);
std::pair<std::vector<cv::KeyPoint>, cv::Mat> single_SIFT(const cv::Mat &img);
std::pair<std::vector<cv::KeyPoint>, cv::Mat> single_BRISK(const cv::Mat &img);

std::vector<cv::DMatch> descriptor_matcher(const cv::Mat &descriptors1,
                                           const cv::Mat &descriptors2,
                                           const float thresh = 0.5f);

std::vector<cv::DMatch> descriptor_matcher(
    const cv::Mat &descriptors1, const std::vector<cv::KeyPoint> &keypoints1,
    const cv::Mat &descriptors2, const std::vector<cv::KeyPoint> &keypoints2,
    const float thresh, const float maxSlope);

std::vector<cv::DMatch> filtered_descriptor_matcher(
    const cv::Mat &descriptors1, const cv::Mat &descriptors2,
    const std::vector<cv::KeyPoint> &keypoints1,
    const std::vector<cv::KeyPoint> &keypoints2, const float thresh);

void draw_and_show(const cv::Mat &imgL, const cv::Mat &imgR, int alg);

void draw_and_show(const cv::Mat &imgL, const cv::Mat &imgR,
                   const std::vector<cv::KeyPoint> &kpsL,
                   const std::vector<cv::KeyPoint> &kpsR,
                   const std::vector<cv::DMatch> &matches);

void draw_landmark_kyp(const cv::Mat &img,
                       const std::vector<Landmark> &landmarks);

std::vector<cv::Point3f> img_to_3dpoints(Calibration &calib,
                                         const cv::Mat &leftimg,
                                         const cv::Mat &rightimg, int alg);

std::vector<Landmark> img_to_landmark(Calibration &calib,
                                      const cv::Mat &leftimg,
                                      const cv::Mat &rightimg, int alg);

std::vector<cv::DMatch> img_to_matches(Calibration &calib,
                                       const cv::Mat &leftimg,
                                       const cv::Mat &rightimg, int alg);

std::vector<cv::Point3f>
stereo_3Dpoints(const cv::Mat &P1, const cv::Mat &P2,
                const std::vector<cv::KeyPoint> &keypoints1,
                const std::vector<cv::KeyPoint> &keypoints2,
                const std::vector<cv::DMatch> &matches);

std::vector<Landmark>
stereo_landmarks(const cv::Mat &P1, const cv::Mat &P2,
                 const std::vector<cv::KeyPoint> &keypoints1,
                 const std::vector<cv::KeyPoint> &keypoints2,
                 const cv::Mat &descriptors1, const cv::Mat &descriptors2,
                 const std::vector<cv::DMatch> &matches);

std::vector<Landmark>
stereo_landmarks(const cv::Mat &P1, const cv::Mat &P2,
                 const std::vector<cv::KeyPoint> &keypoints1,
                 const std::vector<cv::KeyPoint> &keypoints2,
                 const cv::Mat &descriptors1, const cv::Mat &descriptors2,
                 const std::vector<cv::DMatch> &matchesLR,
                 const std::vector<cv::DMatch> &matchesFrames);

std::vector<Landmark> stereo_landmarks(
    Calibration &calib, const std::vector<cv::KeyPoint> &keypoints1,
    const std::vector<cv::KeyPoint> &keypoints2, const cv::Mat &descriptors1,
    const cv::Mat &descriptors2, const std::vector<cv::DMatch> &matches);

std::vector<Landmark> stereo_landmarks(
    Calibration &calib, const std::vector<cv::KeyPoint> &keypoints1,
    const std::vector<cv::KeyPoint> &keypoints2, const cv::Mat &descriptors1,
    const cv::Mat &descriptors2, const std::vector<cv::DMatch> &matchesLR,
    const std::vector<cv::DMatch> &matchesFrames);
