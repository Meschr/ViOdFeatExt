#include "ekf.h"

// Index helpers
#define IDX_P 0
#define IDX_V 3
#define IDX_Q_ERR 6 // we keep quaternion separately, but P's orientation error is 3
#define IDX_BG 9
#define IDX_BA 12

// Utility: convert small-angle vector (theta) to quaternion (approx)
Eigen::Quaterniond smallAngleToQuat(const Eigen::Vector3d &theta)
{
  double theta_norm2 = theta.squaredNorm();
  Eigen::Quaterniond dq;
  if (theta_norm2 * theta_norm2 / 24.0 < 1e-10)
  {
    // use series approx for small angles
    dq.w() = 1.0 - 0.5 * theta_norm2 / 2.0;
    dq.vec() = 0.5 * theta;
  }
  else
  {
    double angle = theta.norm();
    dq.w() = std::cos(angle / 2.0);
    dq.vec() = theta.normalized() * std::sin(angle / 2.0);
  }
  dq.normalize();
  return dq;
}

// Quaternion multiply: q_out = q * r
inline Eigen::Quaterniond quatMultiply(const Eigen::Quaterniond &q, const Eigen::Quaterniond &r)
{
  return q * r;
}

// Skew symmetric matrix from vector
static Eigen::Matrix3d skew(const Eigen::Vector3d &v)
{
  Eigen::Matrix3d m;
  m << 0, -v.z(), v.y(),
      v.z(), 0, -v.x(),
      -v.y(), v.x(), 0;
  return m;
}

EKF::EKF(States_cov_init init_cov, Meas_cov meas_cov, Proc_cov proc_cov)
{
  // initialize to zeros / identity
  p.setZero();
  v.setZero();
  q = Eigen::Quaterniond::Identity();
  bg.setZero();
  ba.setZero();

  P.setZero();
  P.block<3, 3>(IDX_P, IDX_P) = Eigen::Matrix3d::Identity() * init_cov.pos; // small initial pos cov
  P.block<3, 3>(IDX_V, IDX_V) = Eigen::Matrix3d::Identity() * init_cov.pos;
  P.block<3, 3>(IDX_Q_ERR, IDX_Q_ERR) = Eigen::Matrix3d::Identity() * init_cov.quat;
  P.block<3, 3>(IDX_BG, IDX_BG) = Eigen::Matrix3d::Identity() * init_cov.gyro_bias;
  P.block<3, 3>(IDX_BA, IDX_BA) = Eigen::Matrix3d::Identity() * init_cov.acc_bias;

  // default measurement covariances
  R_vo_p     = Eigen::Matrix3d::Identity() * meas_cov.pos;   // tune
  R_vo_theta = Eigen::Matrix3d::Identity() * meas_cov.theta; // tune

  // process noise (continuous) - tune to your sensor
  Q_gyro = Eigen::Matrix3d::Identity() * proc_cov.gyro;
  Q_acc  = Eigen::Matrix3d::Identity() * proc_cov.acc;
  Q_bg   = Eigen::Matrix3d::Identity() * proc_cov.bg;
  Q_ba   = Eigen::Matrix3d::Identity() * proc_cov.ba;

  p_last_vo.setZero();
  q_last_vo = Eigen::Quaterniond::Identity();
  has_last_vo = false;
}

void EKF::setLastVOFrame()
{
  p_last_vo = p;
  q_last_vo = q;
  has_last_vo = true;
}

// Predict step using IMU measurements (acc_m, gyro_m) in body frame
void EKF::predict(const Eigen::Vector3d &acc_m,
                  const Eigen::Vector3d &gyro_m,
                  double dt)
{
  // 1) Remove biases
  Eigen::Vector3d omega = gyro_m - bg;
  Eigen::Vector3d acc = acc_m - ba;

  // 2) Integrate orientation (quaternion) (simple first-order integration)
  Eigen::Vector3d dtheta = omega * dt; // small-angle approx
  Eigen::Quaterniond dq = smallAngleToQuat(dtheta);
  q = quatMultiply(q, dq);
  q.normalize();

  // 3) Integrate velocity & position (assuming acc in body frame)
  Eigen::Vector3d acc_world = q * acc; // rotate body->world
  Eigen::Vector3d g(0, 0, -g_norm);    // gravity in world frame (z down). adjust convention if needed

  v += (acc_world + g) * dt;
  p += v * dt + 0.5 * (acc_world + g) * dt * dt;

  // 4) Linearize and propagate covariance
  Eigen::Matrix<double, STATE_SZ, STATE_SZ> F =
      Eigen::Matrix<double, STATE_SZ, STATE_SZ>::Zero();
  Eigen::Matrix<double, STATE_SZ, 12> G =
      Eigen::Matrix<double, STATE_SZ, 12>::Zero();

  // dp_dot = dv
  F.block<3, 3>(IDX_P, IDX_V) = Eigen::Matrix3d::Identity();

  // dv_dot depends on orientation and accelerometer error
  Eigen::Matrix3d Rwb = q.toRotationMatrix();
  F.block<3, 3>(IDX_V, IDX_Q_ERR) = -Rwb * skew(acc); // effect of small orientation error
  F.block<3, 3>(IDX_V, IDX_BA) = -Rwb;                // acc bias effect

  // dtheta_dot ~ -(omega - bg)_x dtheta - dbg
  F.block<3, 3>(IDX_Q_ERR, IDX_Q_ERR) = -skew(omega);
  F.block<3, 3>(IDX_Q_ERR, IDX_BG) = -Eigen::Matrix3d::Identity();

  // Noise mapping
  G.block<3, 3>(IDX_V, 0) = Rwb;                              // acc noise
  G.block<3, 3>(IDX_Q_ERR, 3) = -Eigen::Matrix3d::Identity(); // gyro noise
  G.block<3, 3>(IDX_BG, 6) = Eigen::Matrix3d::Identity();     // gyro bias noise
  G.block<3, 3>(IDX_BA, 9) = Eigen::Matrix3d::Identity();     // accel bias noise

  // Discretize (first-order)
  Eigen::Matrix<double, STATE_SZ, STATE_SZ> Fd =
      Eigen::Matrix<double, STATE_SZ, STATE_SZ>::Identity() + F * dt;

  Eigen::Matrix<double, 12, 12> Qc =
      Eigen::Matrix<double, 12, 12>::Zero();
  Qc.block<3, 3>(0, 0) = Q_acc;
  Qc.block<3, 3>(3, 3) = Q_gyro;
  Qc.block<3, 3>(6, 6) = Q_bg;
  Qc.block<3, 3>(9, 9) = Q_ba;

  Eigen::Matrix<double, STATE_SZ, STATE_SZ> Qd =
      Fd * G * Qc * G.transpose() * Fd.transpose() * dt;

  P = Fd * P * Fd.transpose() + Qd;
  P = 0.5 * (P + P.transpose());
}

// Relative VO update
void EKF::updateRelativeVO(const Eigen::Vector3d &dp_vo_world,
                           const Eigen::Quaterniond &dq_vo)
{
  if (!has_last_vo)
  {
    setLastVOFrame();
    return;
  }

  // Predicted relative pose
  Eigen::Vector3d dp_imu = p - p_last_vo;
  Eigen::Quaterniond dq_imu = q * q_last_vo.conjugate();

  // Innovations
  Eigen::Vector3d innov_dp = dp_vo_world - dp_imu;

  Eigen::Quaterniond dq_err = dq_vo * dq_imu.conjugate();
  if (dq_err.w() < 0.0)
    dq_err.coeffs() *= -1.0;
  Eigen::Vector3d innov_dtheta = 2.0 * dq_err.vec();

  Eigen::Matrix<double, 6, STATE_SZ> H =
      Eigen::Matrix<double, 6, STATE_SZ>::Zero();
  H.block<3, 3>(0, IDX_P) = Eigen::Matrix3d::Identity();
  H.block<3, 3>(3, IDX_Q_ERR) = Eigen::Matrix3d::Identity();

  Eigen::Matrix<double, 6, 6> R =
      Eigen::Matrix<double, 6, 6>::Zero();
  R.block<3, 3>(0, 0) = R_vo_p;
  R.block<3, 3>(3, 3) = R_vo_theta;

  Eigen::Matrix<double, STATE_SZ, 6> K =
      P * H.transpose() * (H * P * H.transpose() + R).inverse();

  Eigen::Matrix<double, 6, 1> innov;
  innov.block<3, 1>(0, 0) = innov_dp;
  innov.block<3, 1>(3, 0) = innov_dtheta;

  Eigen::VectorXd delta_x = K * innov;

  Eigen::Vector3d dp = delta_x.segment<3>(IDX_P);
  Eigen::Vector3d dv = delta_x.segment<3>(IDX_V);
  Eigen::Vector3d dtheta = delta_x.segment<3>(IDX_Q_ERR);
  Eigen::Vector3d dbg = delta_x.segment<3>(IDX_BG);
  Eigen::Vector3d dba = delta_x.segment<3>(IDX_BA);

  p += dp;
  v += dv;
  q = smallAngleToQuat(dtheta) * q;
  q.normalize();
  bg += dbg;
  ba += dba;

  Eigen::Matrix<double, STATE_SZ, STATE_SZ> I =
      Eigen::Matrix<double, STATE_SZ, STATE_SZ>::Identity();
  Eigen::Matrix<double, STATE_SZ, STATE_SZ> KH = K * H;

  P = (I - KH) * P * (I - KH).transpose() + K * R * K.transpose();
  P = 0.5 * (P + P.transpose());

  setLastVOFrame();
}
