#pragma once

#if __has_include(<Eigen/Dense>)
#include <Eigen/Dense>
#elif __has_include(<eigen3/Eigen/Dense>)
#include <eigen3/Eigen/Dense>
#else
#error "Eigen/Dense not found. Add Eigen include directory to compiler include path (e.g. via CMAKE_PREFIX_PATH or include_directories) or install Eigen."
#endif

#if __has_include(<Eigen/Geometry>)
#include <Eigen/Geometry>
#elif __has_include(<eigen3/Eigen/Geometry>)
#include <eigen3/Eigen/Geometry>
#else
#error "Eigen/Geometry not found. Add Eigen include directory to compiler include path (e.g. via CMAKE_PREFIX_PATH or include_directories) or install Eigen."
#endif
#include <iostream>

// Measurement noice covariance from the visual odometry
struct States_cov_init
{
  float pos       = 1e-3;
  float quat      = 1e-3;
  float gyro_bias = 1e-4;
  float acc_bias  = 1e-4;
};

// Measurement noice covariance from the visual odometry
struct Meas_cov
{
  float pos   = 1e-2; 
  float theta = 1e-3;
};

// Measurement noice covariance from the visual odometry
struct Proc_cov
{
  float gyro = 1e-5;
  float acc  = 1e-3;
  float bg   = 1e-8;
  float ba   = 1e-6;
};

// State sizes
constexpr int P_SZ = 3;
constexpr int V_SZ = 3;
constexpr int Q_SZ = 4; // quaternion (stored as w,x,y,z)
constexpr int BG_SZ = 3;
constexpr int BA_SZ = 3;
constexpr int STATE_SZ = P_SZ + V_SZ + 3 + BG_SZ + BA_SZ; // note: orientation error linearization uses 3 instead of 4

Eigen::Quaterniond smallAngleToQuat(const Eigen::Vector3d &theta);
// Eigen::Quaterniond quatMultiply(const Eigen::Quaterniond &q, const Eigen::Quaterniond &r);
// Eigen::Matrix3d skew(const Eigen::Vector3d &v);

class EKF
{
  private:

public:
  // Full state
  Eigen::Vector3d p;    // position
  Eigen::Vector3d v;    // velocity
  Eigen::Quaterniond q; // orientation (world <- body)
  Eigen::Vector3d bg;   // gyro bias
  Eigen::Vector3d ba;   // accel bias

  // Covariance P for error-state: [dp(3), dv(3), dtheta(3), dbg(3), dba(3)]
  Eigen::Matrix<double, STATE_SZ, STATE_SZ> P;

  // Noise covariances (tunable)
  Eigen::Matrix3d R_vo_p;     // visual relative position measurement covariance
  Eigen::Matrix3d R_vo_theta; // visual relative orientation measurement covariance (small-angle)
  Eigen::Matrix3d Q_gyro;     // gyro noise (continuous)
  Eigen::Matrix3d Q_acc;      // acc noise (continuous)
  Eigen::Matrix3d Q_bg;       // gyro bias random walk
  Eigen::Matrix3d Q_ba;       // acc bias random walk
  double g_norm = 9.80665;

  // For relative VO: last VO-aligned pose
  Eigen::Vector3d p_last_vo;
  Eigen::Quaterniond q_last_vo;
  bool has_last_vo = false;
  EKF(States_cov_init init_cov=States_cov_init(), Meas_cov meas_cov=Meas_cov(), Proc_cov proc_cov=Proc_cov());
  void setLastVOFrame();
  void predict(const Eigen::Vector3d &acc,
               const Eigen::Vector3d &omega,
               double dt);
  void updateRelativeVO(const Eigen::Vector3d &dp_vo_world,
                        const Eigen::Quaterniond &dq_vo);
};