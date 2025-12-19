#include "Transformations.h"
#include <opencv2/core/eigen.hpp>

struct PointToPointResidual {
  PointToPointResidual(const Eigen::Vector3d& p, const Eigen::Vector3d& q) : p_(p), q_(q) {}

  template <typename T>
  bool operator()(const T* const q_rotation,
                  const T* const t_translation,
                  T* residuals) const {

    Eigen::Quaternion<T> q(q_rotation[0],
                            q_rotation[1],
                            q_rotation[2],
                            q_rotation[3]);

    Eigen::Matrix<T,3,1> t(t_translation[0],
                            t_translation[1],
                            t_translation[2]);

    Eigen::Matrix<T,3,1> p = p_.cast<T>();
    Eigen::Matrix<T,3,1> q_target = q_.cast<T>();

    Eigen::Matrix<T,3,1> p_transformed = q * p + t;

    residuals[0] = p_transformed[0] - q_target[0];
    residuals[1] = p_transformed[1] - q_target[1];
    residuals[2] = p_transformed[2] - q_target[2];

    return true;
  }

  Eigen::Vector3d p_;
  Eigen::Vector3d q_;
};

bool estimateRigidCeres(const std::vector<cv::Point3f> &src,
                        const std::vector<cv::Point3f> &dst,
                        cv::Mat &R, cv::Mat &t)
{
  if (src.size() != dst.size() || src.size() < 3) return false;
  // Convert to Eigen
  Eigen::Matrix3d eigenR;
  cv::cv2eigen(R, eigenR);
  Eigen::Quaterniond q_(eigenR);
  q_.normalize();

  double q_rotation[4] = {q_.w(), q_.x(), q_.y(), q_.z()};
  double t_translation[3] = {t.at<double>(0), t.at<double>(1), t.at<double>(2)};
  
  ceres::Problem problem;
  ceres::LossFunction* loss = new ceres::HuberLoss(0.1);
  for (size_t i = 0; i < src.size(); ++i) {
    ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<
                                              PointToPointResidual, 3, 4, 3>(
                                                new PointToPointResidual(
                                                  Eigen::Vector3d(src[i].x, src[i].y, src[i].z), 
                                                  Eigen::Vector3d(dst[i].x, dst[i].y, dst[i].z))
                                              );

    problem.AddResidualBlock(cost_function,
                             loss,
                             q_rotation,
                             t_translation);
  }

  // Enforce unit quaternion
  problem.SetParameterization(q_rotation, new ceres::QuaternionParameterization());

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 10;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  // std::cout << "Termination type: " << summary.termination_type << std::endl;
  if (summary.termination_type != ceres::CONVERGENCE || summary.final_cost > 1.0) {
    std::cout << summary.BriefReport() << std::endl;
    return false;
  }
  std::cout << "Final cost: " << summary.final_cost << std::endl << std::endl;
  // if (!summary.IsSolutionUsable()) return false;

  // Convert back to cv::Mat
  eigenR = Eigen::Quaterniond(q_rotation[0], q_rotation[1], q_rotation[2], q_rotation[3]).toRotationMatrix();
  cv::eigen2cv(eigenR, R);
  t.at<double>(0) = t_translation[0];
  t.at<double>(1) = t_translation[1];
  t.at<double>(2) = t_translation[2];

  return true;
}

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
  if (cv::norm(t) > 0.5) 
    return false; // unreasonable translation

  return true;
}

bool estimateRigidRANSAC(const std::vector<cv::Point3f> &src,
                         const std::vector<cv::Point3f> &dst,
                         cv::Mat &R, cv::Mat &t,
                         int iterations, float threshold)
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
                                cv::Affine3d &transformation,
                                const TransformationType type){
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

  cv::Mat R = cv::Mat::eye(3, 3, CV_32F); 
  cv::Mat t = cv::Mat::zeros(3, 1, CV_32F);

  switch (type)
  {
  case UNDEFINED_TYPE:
    std::cerr << "RANSAC failed to estimate transformation.\n";
    return false;
  case RANSAC_SVD_CERES:
  case RANSAC_SVD:
    if (!estimateRigidRANSAC(positions_last, positions_current, R, t))
    {
      std::cerr << "RANSAC failed to estimate transformation.\n";
      return false;
    }
    if(RANSAC_SVD == type) break;
  case CERES:
    if(!estimareRigidCeres(positions_last, positions_current, R, t))
    {
      std::cerr << "Ceres failed to estimate transformation.\n";
      return false;
    }
    if(CERES == type) break;
  default:
    break;
  }

  transformation = cv::Affine3d(R, t); 
  return true;
}