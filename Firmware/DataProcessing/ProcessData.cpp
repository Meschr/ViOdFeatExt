#include "ProcessData.h"
#include <iostream>


std::vector<cv::KeyPoint> single_FAST(const cv::Mat &img)
{
  auto fast = cv::FastFeatureDetector::create(50);
  std::vector<cv::KeyPoint> keypoints;

  fast->detect(img, keypoints);

  return keypoints;
}





std::pair<std::vector<cv::KeyPoint>, cv::Mat> single_ORB(const cv::Mat &img)
{
  auto orb = cv::ORB::create(100);
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
                   const std::vector<cv::DMatch> &matches,
                   const std::vector<Landmark> &landmarks)
{
  cv::Mat img_matches;
  cv::drawMatches(imgL, keypoints1, imgR, keypoints2, matches, img_matches);


  cv::Mat small_matches;
  cv::resize(img_matches, small_matches, cv::Size(), 1.5, 1.5);
  cv::imshow("ORB Feature Match", small_matches);
}




void draw_landmark_kyp(const cv::Mat &img, 
                       const std::vector<Landmark> &landmarks)
{
    cv::Mat img_draw = img.clone();
    for (const auto &lm : landmarks)
    {
        cv::circle(img_draw, lm.keypoint.pt, 4, cv::Scalar(0,255,255), -1);
        cv::putText(img_draw, lm.id, lm.keypoint.pt + cv::Point2f(5,-5),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,0), 1, cv::LINE_AA);
    }
    cv::imshow("Landmark Keypoints", img_draw);
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

  cv::Mat P1f, P2f;
  P1.convertTo(P1f, CV_32F);
  P2.convertTo(P2f, CV_32F);

  cv::triangulatePoints(P1f, P2f, pts1, pts2, points4D);

  points3D.reserve(points4D.cols);

  for (int i = 0; i < points4D.cols; ++i)
  {
    cv::Mat col = points4D.col(i);
    float X = col.at<float>(0) / col.at<float>(3);
    float Y = col.at<float>(1) / col.at<float>(3);
    float Z = col.at<float>(2) / col.at<float>(3); // <-- depth from left camera

    if (!std::isfinite(Z) || Z <= 0.0f)
      continue;
      
    points3D.emplace_back(X, Y, Z);
  }

  return points3D;
}



std::vector<Landmark> path_to_landmark(
        const std::string &leftPath,
        const std::string &rightPath,
        const cv::Mat &P1,
        const cv::Mat &P2)
{
    cv::Mat imgL = cv::imread(leftPath, cv::IMREAD_GRAYSCALE);
    cv::Mat imgR = cv::imread(rightPath, cv::IMREAD_GRAYSCALE);

    auto [kpsL, dL] = single_ORB(imgL);
    auto [kpsR, dR] = single_ORB(imgR);

    auto stereoMatches = descriptor_matcher(dL, dR, 0.7);
    return stereo_landmarks(P1, P2, kpsL, kpsR, dL, dR, stereoMatches);
}




std::vector<Landmark> stereo_landmarks(const cv::Mat &P1,
                                       const cv::Mat &P2,
                                       const std::vector<cv::KeyPoint> &keypoints1,
                                       const std::vector<cv::KeyPoint> &keypoints2,
                                       const cv::Mat &descriptors1,
                                       const cv::Mat &descriptors2,
                                       const std::vector<cv::DMatch> &matches)
{
  std::vector<Landmark> landmarks;
  std::vector<cv::Point2f> pts1, pts2;
  cv::Mat points4D;


  if (matches.empty() ||
      keypoints1.empty() ||
      keypoints2.empty() ||
      descriptors1.empty() ||
      descriptors2.empty())
    return landmarks;


  pts1.reserve(matches.size());
  pts2.reserve(matches.size());

  for (const auto &m : matches)
  {
    pts1.push_back(keypoints1[m.queryIdx].pt);
    pts2.push_back(keypoints2[m.trainIdx].pt);
  }

  cv::Mat P1f, P2f;
  P1.convertTo(P1f, CV_32F);
  P2.convertTo(P2f, CV_32F);

  cv::triangulatePoints(P1f, P2f, pts1, pts2, points4D);

  landmarks.reserve(points4D.cols);
  
  static int frame_num = 1; 
  static int local_id = 1;


  for (int i = 0; i < points4D.cols; ++i)
  {
    // Dehomogenize (assuming float; change to double if P1/P2 are CV_64F)
    float X = points4D.at<float>(0, i) / points4D.at<float>(3, i);
    float Y = points4D.at<float>(1, i) / points4D.at<float>(3, i);
    float Z = points4D.at<float>(2, i) / points4D.at<float>(3, i);

    if (!std::isfinite(Z) || Z <= 0.0f)
      continue;

    const cv::DMatch &m = matches[i];

    // Choose descriptor from the keypoint with higher response
    const cv::KeyPoint &kp1 = keypoints1[m.queryIdx];
    const cv::KeyPoint &kp2 = keypoints2[m.trainIdx];

    cv::Mat chosenDesc = (kp1.response >= kp2.response) ? descriptors1.row(m.queryIdx) : descriptors2.row(m.trainIdx);

    Landmark lm = {.position = cv::Point3f(X, Y, Z),
                   .descriptor = chosenDesc.clone(),
                   .id         = std::to_string(frame_num) + "_" + std::to_string(local_id++),
                   .keypoint   = kp1};

    landmarks.push_back(std::move(lm));
  }

  return landmarks;
}
