#include "CaptureDevice.h"
#include "Calibration.h"
#include "ProcessData.h"
#include "DatasetReader.h"
#include <iostream>
#include <fstream>

bool estimateRigidSVD(const std::vector<cv::Point3f> &src,
                      const std::vector<cv::Point3f> &dst,
                      cv::Mat &R, cv::Mat &t)
{
  if (src.size() != dst.size() || src.size() < 3) return false;
  const int N = static_cast<int>(src.size());

  cv::Mat srcMat(N, 3, CV_64F), dstMat(N, 3, CV_64F);
  for (int i = 0; i < N; ++i) {
    srcMat.at<double>(i,0) = src[i].x;
    srcMat.at<double>(i,1) = src[i].y;
    srcMat.at<double>(i,2) = src[i].z;
    dstMat.at<double>(i,0) = dst[i].x;
    dstMat.at<double>(i,1) = dst[i].y;
    dstMat.at<double>(i,2) = dst[i].z;
  }

  cv::Mat centroidSrc, centroidDst;
  cv::reduce(srcMat, centroidSrc, 0, cv::REDUCE_AVG);
  cv::reduce(dstMat, centroidDst, 0, cv::REDUCE_AVG);

  cv::Mat srcZero = srcMat - cv::repeat(centroidSrc, N, 1);
  cv::Mat dstZero = dstMat - cv::repeat(centroidDst, N, 1);
  cv::Mat H = srcZero.t() * dstZero;
  cv::SVD svd(H);

  R = svd.vt.t() * svd.u.t();

  // Ensure right-handed
  if (cv::determinant(R) < 0) {
    cv::Mat vt = svd.vt.clone();
    vt.row(2) *= -1;
    R = vt.t() * svd.u.t();
  }

  t = centroidDst.t() - R * centroidSrc.t();
  return true;
}

bool estimateRigidRANSAC(const std::vector<cv::Point3f> &src,
                         const std::vector<cv::Point3f> &dst,
                         cv::Mat &R, cv::Mat &t,
                         int iterations = 1000,
                         float threshold = 0.05f)
{
  CV_Assert(src.size() == dst.size());
  const int N = static_cast<int>(src.size());
  if (N < 3) return false;

  cv::RNG rng;
  int bestInliers = 0;
  cv::Mat bestR, bestT;
  std::vector<int> bestInlierIdx;

  for (int it = 0; it < iterations; ++it) {
    // sample 3 distinct indices
    std::array<int,3> idx{};
    for (int k = 0; k < 3;) {
      int r = rng.uniform(0, N);
      bool dup = false;

      // check for duplicates
      for (int j = 0; j < k; ++j) 
        dup |= (idx[j] == r);
      if (!dup) 
        idx[k++] = r;
    }

    std::vector<cv::Point3f> s = {src[idx[0]], src[idx[1]], src[idx[2]]};
    std::vector<cv::Point3f> d = {dst[idx[0]], dst[idx[1]], dst[idx[2]]};

    cv::Mat Rm, tm;
    if (!estimateRigidSVD(s, d, Rm, tm)) continue;

    std::vector<int> inliers;
    inliers.reserve(N);
    for (int i = 0; i < N; ++i) {
      cv::Mat pt = (cv::Mat_<double>(3,1) << src[i].x, src[i].y, src[i].z);
      cv::Mat proj = Rm * pt + tm;
      cv::Point3f projected((float)proj.at<double>(0,0), (float)proj.at<double>(1,0), (float)proj.at<double>(2,0));
      float err = cv::norm(projected - dst[i]);
      if (err < threshold) inliers.push_back(i);
    }

    if ((int)inliers.size() > bestInliers) {
      bestInliers = (int)inliers.size();
      bestR = Rm.clone();
      bestT = tm.clone();
      bestInlierIdx.swap(inliers);
    }
    // early exit if model explains most points
    if (bestInliers > 0.9 * N)
      break;
  }

  if (bestInliers < 3 || bestInlierIdx.empty()) {
    return false; // no good model
  }

  // refine on all inliers of best model
  std::vector<cv::Point3f> s_in, d_in;
  s_in.reserve(bestInliers);
  d_in.reserve(bestInliers);
  for (int i : bestInlierIdx) {
    s_in.push_back(src[i]);
    d_in.push_back(dst[i]);
  }
  if (!estimateRigidSVD(s_in, d_in, R, t)) return false;

  // recompute inliers for the refined model and ensure it's still good
  int finalInliers = 0;
  for (int i = 0; i < N; ++i) {
    cv::Mat pt = (cv::Mat_<double>(3,1) << src[i].x, src[i].y, src[i].z);
    cv::Mat proj = R * pt + t;
    cv::Point3f projected((float)proj.at<double>(0,0), (float)proj.at<double>(1,0), (float)proj.at<double>(2,0));
    float err = cv::norm(projected - dst[i]);
    if (err < threshold) ++finalInliers;
  }
  if (finalInliers < 3) return false;
  return true;
}

bool transformation_calculation(const std::vector<Landmark> &landmarks,
                                const std::vector<Landmark> &last_landmarks,
                                cv::Affine3d &transformation){
  cv::Mat descriptors_current, descriptors_last;
  cv::Mat positions_current, positions_last;

  for (const auto &lm : landmarks)
    descriptors_current.push_back(lm.descriptor.clone());
     
  for (const auto &lm : last_landmarks)
    descriptors_last.push_back(lm.descriptor.clone());

  auto matches = descriptor_matcher(descriptors_current, descriptors_last, 0.5f);
  if(matches.empty()) return false;
  std::cout << "Number of matches with last frame: " << matches.size() << std::endl;

  positions_current.reserve(matches.size());
  positions_last.reserve(matches.size());

  for (const auto &m : matches)
  {
    positions_current.push_back(landmarks[m.queryIdx].position);
    positions_last.push_back(last_landmarks[m.trainIdx].position);
  }
  //std::cout << "Previous positions: " << positions_last << "\n";
  //std::cout << "current positions: " << positions_current << "\n";

  cv::Mat R, t;
  if (!estimateRigidRANSAC(positions_last, positions_current, R, t))
  {
    std::cerr << "RANSAC failed to estimate transformation.\n";
    return false;
  }
  transformation = cv::Affine3d(R, t); 
  return true;
}

int main(int argc, char **argv)
{

  DatasetReader reader;
  Calibration calib;
  std::string filename = "../DataProcessing/stereoCalibration.yml";
  calib.loadStereoCalibration(filename);

  const std::string subfolderName = "2025-11-18_11-59-45_linearMovementPointedIntoTheRoom400mm";
  
  std::ofstream csvFile("../tests/translations.csv", std::ios::app);

  reader.loadImagePairs(subfolderName, false);

  if (argc > 1)
    filename = argv[1];

  try
  {
    // camDev.Init();
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << '\n';
    return 1;
  }

  std::vector<Landmark> last_landmarks;

  cv::Mat imgL_raw, imgR_raw, imgL, imgR, lastR, lastL;
  int frame_counter = 0;

  //-------------------------------------------------k --> 1 = ORB, 2 = BRISK------------------------------------------------------------------------------
  int k = 2;
  //--------------------------------------------------- resolution = 0.5 means half the size of the image----------------------------------------
  float resolution = 1.;

  while (reader.nextStereoImagePair(imgL_raw, imgR_raw, resolution))
  {

    frame_counter++;
    // Skip frames that are not every 5th
    if (frame_counter % 5 != 0)
      continue;

    calib.rectifyStereo(imgL_raw, imgR_raw, imgL, imgR);
    auto landmarks = img_to_landmark(calib, imgL, imgR, k);

    std::cout << "Frame: " << frame_counter << std::endl;

    cv::Affine3d transformation;
    if (!last_landmarks.empty() && transformation_calculation(landmarks, last_landmarks, transformation))
    {
      // std::cout << "Estimated Transformation:\n" << transformation.translation() << std::endl;
      cv::Vec3d translation = transformation.translation();
      csvFile << translation[0] << "," << translation[1] << "," << translation[2] << "\n";
    }else{
      csvFile << "0, 0, 0\n";
    }

    last_landmarks.clear();
    last_landmarks.reserve(landmarks.size());
    for (auto &lm : landmarks) {
        Landmark copy = lm;
        copy.descriptor = lm.descriptor.clone();
        last_landmarks.push_back(copy);
    }
  }
  csvFile.close();
  return 0;
}
