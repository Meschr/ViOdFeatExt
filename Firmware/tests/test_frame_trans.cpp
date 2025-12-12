#include "CaptureDevice.h"
#include "Calibration.h"
#include "ProcessData.h"
#include "DatasetReader.h"
#include <iostream>
#include <fstream>

bool estimateRigidSVD(
    const std::vector<cv::Point3f> &src,
    const std::vector<cv::Point3f> &dst,
    cv::Mat &R, cv::Mat &t)
{
  int N = src.size();
  if (N < 3)
    return false;

  cv::Mat srcMat(N, 3, CV_64F), dstMat(N, 3, CV_64F);

  for (int i = 0; i < N; i++)
  {
    srcMat.at<double>(i, 0) = src[i].x;
    srcMat.at<double>(i, 1) = src[i].y;
    srcMat.at<double>(i, 2) = src[i].z;

    dstMat.at<double>(i, 0) = dst[i].x;
    dstMat.at<double>(i, 1) = dst[i].y;
    dstMat.at<double>(i, 2) = dst[i].z;
  }

  cv::Mat centroidSrc, centroidDst;
  cv::reduce(srcMat, centroidSrc, 0, cv::REDUCE_AVG);
  cv::reduce(dstMat, centroidDst, 0, cv::REDUCE_AVG);

  cv::Mat srcZero = srcMat - cv::repeat(centroidSrc, N, 1);
  cv::Mat dstZero = dstMat - cv::repeat(centroidDst, N, 1);

  cv::Mat H = srcZero.t() * dstZero;
  cv::SVD svd(H);

  R = svd.vt.t() * svd.u.t();

  // Fix reflection
  if (cv::determinant(R) < 0)
  {
    cv::Mat vt = svd.vt.clone();
    vt.row(2) *= -1;
    R = vt.t() * svd.u.t();
  }

  t = centroidDst.t() - R * centroidSrc.t();
  return true;
}

cv::Affine3d estimateRigidRANSAC(
    const std::vector<cv::Point3f> &src,
    const std::vector<cv::Point3f> &dst,
    int iterations = 200,
    float threshold = 0.03f) // 3 cm threshold (adjust!)
{
  std::cout << "size src: " << src.size() << "\n";
  std::cout << "size dst: " << dst.size() << "\n";

  CV_Assert(src.size() == dst.size());
  int N = src.size();

  cv::RNG rng;
  int bestInliers = 0;
  cv::Mat bestR, bestT;

  for (int it = 0; it < iterations; it++)
  {
    // --- Random minimal sample ---
    std::vector<cv::Point3f> s, d;
    for (int i = 0; i < 3; i++)
    {
      int idx = rng.uniform(0, N);
      s.push_back(src[idx]);
      d.push_back(dst[idx]);
    }

    cv::Mat R, t;
    if (!estimateRigidSVD(s, d, R, t))
      continue;

    // --- Count inliers ---
    int inliers = 0;
    for (int i = 0; i < N; i++)
    {
      cv::Mat pt = (cv::Mat_<double>(3, 1) << src[i].x, src[i].y, src[i].z);
      cv::Mat proj = R * pt + t;

      cv::Point3f pred(proj.at<double>(0), proj.at<double>(1), proj.at<double>(2));
      float err = cv::norm(pred - dst[i]);

      if (err < threshold)
        inliers++;
    }

    // --- Keep best ---
    if (inliers > bestInliers)
    {
      bestInliers = inliers;
      bestR = R.clone();
      bestT = t.clone();
    }
  }

  // Assemble final transform
  cv::Affine3d T(bestR, bestT);
  return T;
}

int main(int argc, char **argv)
{

  DatasetReader reader;
  Calibration calib;
  std::string filename = "../DataProcessing/stereoCalibration.yml";
  calib.loadStereoCalibration(filename);


  const std::string subfolderName = "2025-11-18_12-18-21_ 2AxisMovementPointedTowardsMoon250mmSquare";

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
  int k = 1;
  //--------------------------------------------------- resolution = 0.5 means half the size of the image----------------------------------------
  float resolution = 1.;

  while (reader.nextStereoImagePair(imgL_raw, imgR_raw, resolution))
  {

    // Skip frames that are not every 10th
    // if (frame_counter % 10 != 0) {
    //     frame_counter++;
    //     continue;
    // }
    calib.rectifyStereo(imgL_raw, imgR_raw, imgL, imgR);

    auto landmarks = img_to_landmark(calib, imgL, imgR, k);

    std::cout << "Frame: " << frame_counter << std::endl;

    if (!last_landmarks.empty())
    {
      cv::Mat descriptors_current, descriptors_last;
      cv::Mat positions_current, positions_last;

      for (const auto &lm : landmarks)
      {
        descriptors_current.push_back(lm.descriptor);
        // positions_current.push_back(lm.position);
      }

      for (const auto &lm : last_landmarks)
      {
        descriptors_last.push_back(lm.descriptor);
        // positions_last.push_back(lm.position);
      }

      auto matches = descriptor_matcher(descriptors_current, descriptors_last, 0.9f);
      if (matches.size() == 0){
        frame_counter ++;
        continue;}

      std::cout << "Number of matches with last frame: " << matches.size() << std::endl;

      positions_current.reserve(matches.size());
      positions_last.reserve(matches.size());

      for (const auto &m : matches)
      {
        positions_last.push_back(last_landmarks[m.queryIdx].position);
        positions_current.push_back(landmarks[m.trainIdx].position);
      }
      std::cout << "Previous positions: " << positions_last << "\n";
      std::cout << "cuurent positions: " << positions_current << "\n";

      cv::Affine3d frame_transform = estimateRigidRANSAC(positions_last, positions_current);
      
      cv::Vec3d t = frame_transform.translation();
      std::ofstream csvFile("../Tests_Algorithms/translations.csv", std::ios::app);
      csvFile << t[0] << "," << t[1] << "," << t[2] << "\n";
      csvFile.close();

      // std::cout << "Estimated frame-to-frame transformation:\n"
      //           << "Rotation:\n"
      //           << frame_transform.rotation() << "\n"
      //           << "Translation:\n"
      //           << frame_transform.translation() << "\n";

    }

      frame_counter++;
      last_landmarks = landmarks;
      // if (cv::waitKey(0) == 'q') {
      //   break; // Exit on 'q'
      // }
  }
  return 0;
}