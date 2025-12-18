#pragma once

#include <ceres/ceres.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include "ProcessData.h"

enum TransformationType {
  UNDEFINED_TYPE,
  RANSAC_SVD,
  CERES,
  RANSAC_SVD_CERES
};

bool estimateRigidSVD(const std::vector<cv::Point3f> &src,
                      const std::vector<cv::Point3f> &dst,
                      cv::Mat &R, cv::Mat &t);

bool estimateRigidRANSAC(const std::vector<cv::Point3f> &src,
                         const std::vector<cv::Point3f> &dst,
                         cv::Mat &R, cv::Mat &t,
                         int iterations = 1000,
                         float threshold = 0.005f);

bool transformation_calculation(const std::vector<Landmark> &current_landmarks,
                                const std::vector<Landmark> &last_landmarks,
                                cv::Affine3d &transformation,
                                const TransformationType type = RANSAC_SVD_CERES);