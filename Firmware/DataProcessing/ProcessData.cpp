#include "ProcessData.h"
#include <iostream>

std::vector<cv::KeyPoint> single_FAST(const cv::Mat &img)
{
  auto fast = cv::FastFeatureDetector::create(30);
  std::vector<cv::KeyPoint> keypoints;

  fast->detect(img, keypoints);

  return keypoints;
}

std::pair<std::vector<cv::KeyPoint>, cv::Mat> single_ORB(const cv::Mat &img)
{
  auto orb = cv::ORB::create(200);
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;

  orb->detectAndCompute(img, cv::noArray(), keypoints, descriptors);

  return {keypoints, descriptors};
}

std::pair<std::vector<cv::KeyPoint>, cv::Mat> single_SIFT(const cv::Mat &img)
{
  auto sift = cv::SIFT::create(30);
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  sift->detectAndCompute(img, cv::noArray(), keypoints, descriptors);
  return {keypoints, descriptors};
}

std::vector<cv::DMatch> descriptor_matcher(const cv::Mat &descriptors1, const cv::Mat &descriptors2, const float thresh)
{
  cv::BFMatcher matcher(cv::NORM_HAMMING);
  std::vector<std::vector<cv::DMatch>> knnMatches;
  std::vector<cv::DMatch> good_matches;

  if (descriptors1.empty() || descriptors2.empty())
    return good_matches;

  matcher.knnMatch(descriptors1, descriptors2, knnMatches, 2);

  for (auto &m : knnMatches)
  {
    if (m.size() < 2)
      continue;
    if (m[0].distance < thresh * m[1].distance)
    { // can change this value to say how much better the best match has to be compared to second best
      good_matches.push_back(m[0]);
    }
  }
  return good_matches;
}

void draw_and_show(const cv::Mat &imgL,
                   const cv::Mat &imgR,
                   const std::vector<cv::KeyPoint> &keypoints1,
                   const std::vector<cv::KeyPoint> &keypoints2,
                   const std::vector<cv::DMatch> &matches)
{
  cv::Mat img_matches;
  cv::drawMatches(imgL, keypoints1, imgR, keypoints2, matches, img_matches);

  cv::Mat small_matches;
  cv::resize(img_matches, small_matches, cv::Size(), 1.5, 1.5);
  cv::imshow("ORB Feature Match", small_matches);
}

std::vector<cv::Point3f> stereo_3Dpoints(const cv::Mat &P1,
                                         const cv::Mat &P2,
                                         const std::vector<cv::KeyPoint> &keypoints1,
                                         const std::vector<cv::KeyPoint> &keypoints2,
                                         const std::vector<cv::DMatch> &matches)
{
  std::vector<cv::Point2f> pts1, pts2;
  cv::Mat points4D;
  std::vector<cv::Point3f> points3D;

  if (matches.empty() || keypoints1.empty() || keypoints2.empty())
    return points3D;

  pts1.reserve(matches.size());
  pts2.reserve(matches.size());

  for (const auto &m : matches)
  {
    pts1.push_back(keypoints1[m.queryIdx].pt);
    pts2.push_back(keypoints2[m.trainIdx].pt);
  }

  cv::triangulatePoints(P1, P2, pts1, pts2, points4D);

  points3D.reserve(points4D.cols);

  for (int i = 0; i < points4D.cols; ++i)
  {
    cv::Mat col = points4D.col(i);
    float X = col.at<float>(0) / col.at<float>(3);
    float Y = col.at<float>(1) / col.at<float>(3);
    float Z = col.at<float>(2) / col.at<float>(3); // <-- depth from left camera

    points3D.emplace_back(X, Y, Z);
  }

  return points3D;
}