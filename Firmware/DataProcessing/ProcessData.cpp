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
  auto orb = cv::ORB::create(50);
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;

  orb->detectAndCompute(img, cv::noArray(), keypoints, descriptors);

  return {keypoints, descriptors};
}





std::pair<std::vector<cv::KeyPoint>, cv::Mat> single_BRISK(const cv::Mat &img)
{
    auto brisk = cv::BRISK::create(90);  
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    brisk->detectAndCompute(img, cv::noArray(), keypoints, descriptors);

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







// for diplaying 2 images side by side ans connecting keypoints
void draw_and_show(const cv::Mat &imgL,
                   const cv::Mat &imgR,
                   int alg)
{
  cv::Mat img_matches;
  std::cout<< alg << std::endl;
  std::vector<cv::KeyPoint> kpsL, kpsR;
  cv::Mat dL, dR;

  if (alg == 1) {
      std::tie(kpsL, dL) = single_ORB(imgL);
      std::tie(kpsR, dR) = single_ORB(imgR);
  }

  if (alg == 2) {
      std::tie(kpsL, dL) = single_BRISK(imgL);
      std::tie(kpsR, dR) = single_BRISK(imgR);
  }

  auto stereoMatches = descriptor_matcher(dL, dR, 0.6);
  cv::drawMatches(imgL, kpsL, imgR, kpsR, stereoMatches, img_matches);
  cv::Mat small_matches;

  cv::resize(img_matches, small_matches, cv::Size(), 1.5, 1.5);

  if (alg == 1){
      cv::imshow("Feature Match with ORB", small_matches);
  }
  if (alg == 2){
      cv::imshow("Feature Match with BRISK", small_matches);
  }
}







// for drawing only on one image the matched keypoints
void draw_landmark_kyp(const cv::Mat &img, 
                       const std::vector<Landmark> &landmarks)
{
    cv::Mat img_draw = img.clone();
    for (const auto &lm : landmarks)
    {
        cv::circle(img_draw, lm.keypoint.pt, 4, cv::Scalar(0,255,255), -1);
        //cv::putText(img_draw, lm.id, lm.keypoint.pt + cv::Point2f(5,-5)),
        //cv::FONT_HERSHEY_SIMPLEX (0.5, cv::Scalar(0,255,0), 1, cv::LINE_AA);
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








// give 2 images and get pack the 3d points 
std::vector<cv::Point3f> img_to_3dpoints(
        Calibration &calib,
        const cv::Mat &leftimg,
        const cv::Mat &rightimg,
        int alg )
{

    std::vector<cv::KeyPoint> kpsL, kpsR;
    cv::Mat dL, dR;

    if (alg == 1) {
        std::tie(kpsL, dL) = single_ORB(leftimg);
        std::tie(kpsR, dR) = single_ORB(rightimg);
    }

    if (alg == 2) {
        std::tie(kpsL, dL) = single_BRISK(leftimg);
        std::tie(kpsR, dR) = single_BRISK(rightimg);
    }

    auto stereoMatches = descriptor_matcher(dL, dR, 0.6);
    auto landmarks = stereo_landmarks(calib, kpsL, kpsR, dL, dR, stereoMatches);

    std::vector<cv::Point3f> positions;
    for (const auto &lm : landmarks) {
      std::cout << lm.position << std::endl;
      positions.push_back(lm.position);
    }

    return positions;
}





// give 2 images and get back the matches
// int algorithm: 1 = ORB, 2 = BRISK, 3 = SIFT?
std::vector<cv::DMatch> img_to_matches(
        Calibration &calib,
        const cv::Mat &leftimg,
        const cv::Mat &rightimg,
        int alg)
{
    std::vector<cv::KeyPoint> kpsL, kpsR;
    cv::Mat dL, dR;

    if (alg == 1) {
        std::tie(kpsL, dL) = single_ORB(leftimg);
        std::tie(kpsR, dR) = single_ORB(rightimg);
    }

    if (alg == 2) {
        std::tie(kpsL, dL) = single_BRISK(leftimg);
        std::tie(kpsR, dR) = single_BRISK(rightimg);
    }

    auto stereoMatches = descriptor_matcher(dL, dR, 0.6);


    return stereoMatches;
}



// give 2 images and get pack the 3d points and the IDs
std::vector<Landmark> img_to_landmark(
        Calibration &calib,
        const cv::Mat &leftimg,
        const cv::Mat &rightimg,
        int alg)
{
    std::vector<cv::KeyPoint> kpsL, kpsR;
    cv::Mat dL, dR;

    if (alg == 1) {
        std::tie(kpsL, dL) = single_ORB(leftimg);
        std::tie(kpsR, dR) = single_ORB(rightimg);
    }

    if (alg == 2) {
        std::tie(kpsL, dL) = single_BRISK(leftimg);
        std::tie(kpsR, dR) = single_BRISK(rightimg);
    }

    auto stereoMatches = descriptor_matcher(dL, dR, 0.7);
    auto landmarks = stereo_landmarks(calib, kpsL, kpsR, dL, dR, stereoMatches);
    return landmarks;
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
                   .keypoint   = kp1};

    landmarks.push_back(std::move(lm));
  }

  return landmarks;
}



std::vector<Landmark> stereo_landmarks(Calibration &calib,
                                       const std::vector<cv::KeyPoint> &keypoints1,
                                       const std::vector<cv::KeyPoint> &keypoints2,
                                       const cv::Mat &descriptors1,
                                       const cv::Mat &descriptors2,
                                       const std::vector<cv::DMatch> &matches)
{
  if (calib.P1.empty() || calib.P2.empty())
    calib.calcRectifyParameters();
  return stereo_landmarks(calib.P1, calib.P2, keypoints1, keypoints2, descriptors1, descriptors2, matches);
}