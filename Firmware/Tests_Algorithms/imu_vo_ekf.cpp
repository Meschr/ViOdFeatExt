#include <iostream>
#include "ekf.h"

// Example usage & test main (simulated relative VO)
int main() {
    EKF ekf;

    // Simulated stream: predict at 200 Hz with IMU, VO updates at 20 Hz
    double imu_dt      = 0.005; // 200 Hz
    double sim_time    = 2.0;
    int imu_steps      = int(sim_time / imu_dt);
    double vo_interval = 0.05;  // 20 Hz
    double next_vo_time = 0.0;

    // For generating relative VO: store previous "true" pose (here: EKF state as proxy)
    Eigen::Vector3d p_prev_vo = ekf.p;
    Eigen::Quaterniond q_prev_vo = ekf.q;

    for (int k = 0; k < imu_steps; ++k) {
        double t = k * imu_dt;

        // IMU measurements (simulate simple motion)
        Eigen::Vector3d acc_m(0.1, 0.0, 0.0);  // body frame acc
        Eigen::Vector3d gyro_m(0.0, 0.0, 0.01); // small yaw rate

        ekf.predict(acc_m, gyro_m, imu_dt);

        if (t >= next_vo_time - 1e-9) {
            // --- Simulate a relative VO measurement between last VO time and now ---

            // "True" relative motion from prev VO pose to current (using current EKF state as proxy)
            Eigen::Vector3d dp_true = ekf.p - p_prev_vo;
            Eigen::Quaterniond dq_true = ekf.q * q_prev_vo.conjugate();

            // Add small noise to translation
            Eigen::Vector3d dp_vo_world = dp_true + Eigen::Vector3d::Random() * 0.01;

            // Add small noise to rotation
            Eigen::Vector3d ang_noise = Eigen::Vector3d::Random() * 0.01;
            Eigen::Quaterniond dq_noise = smallAngleToQuat(ang_noise);
            Eigen::Quaterniond dq_vo = dq_noise * dq_true;
            dq_vo.normalize();

            // Call relative VO update
            ekf.updateRelativeVO(dp_vo_world, dq_vo);

            // Update "prev VO" pose for the next relative measurement
            p_prev_vo = ekf.p;
            q_prev_vo = ekf.q;

            next_vo_time += vo_interval;
        }
    }

    std::cout << "Final state:\n";
    std::cout << "p: " << ekf.p.transpose() << "\n";
    std::cout << "v: " << ekf.v.transpose() << "\n";
    std::cout << "q (w,x,y,z): " << ekf.q.w() << " " << ekf.q.vec().transpose() << "\n";
    std::cout << "bg: " << ekf.bg.transpose() << "\n";
    std::cout << "ba: " << ekf.ba.transpose() << "\n";

    return 0;
}
