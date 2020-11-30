/*
 * @Description: IMU pre-integrator for LIO mapping, implementation
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 */

#include "lidar_localization/models/pre_integrator/imu_pre_integrator.hpp"

namespace lidar_localization {

IMUPreIntegrator::IMUPreIntegrator(void) {
    F_.block<3, 3>(INDEX_ALPHA,  INDEX_BETA) =  Eigen::Matrix3d::Identity();
    F_.block<3, 3>(INDEX_THETA,   INDEX_B_G) = -Eigen::Matrix3d::Identity();

    G_.block<3, 3>(INDEX_THETA, INDEX_THETA) = G_.block<3, 3>(INDEX_THETA, INDEX_B_A) = 0.50 * Eigen::Matrix3d::Identity();
    G_.block<3, 3>(INDEX_B_A, INDEX_B_A) = G_.block<3, 3>(INDEX_B_G, INDEX_B_G) = Eigen::Matrix3d::Identity();
}

/**
 * @brief  reset IMU pre-integrator
 * @param  init_imu_data, init IMU measurements
 * @return true if success false otherwise
 */
bool IMUPreIntegrator::Init(const IMUData &init_imu_data) {
    // reset pre-integrator state:
    ResetState(init_imu_data);
    return true;
}

/**
 * @brief  update IMU pre-integrator
 * @param  imu_data, current IMU measurements
 * @return true if success false otherwise
 */
bool IMUPreIntegrator::Update(const IMUData &init_imu_data) {
    return true;
}

/**
 * @brief  get measurement for IMU pre-integration edge
 * @param  void
 * @return true if success false otherwise
 */
bool IMUPreIntegrator::GetMeasurement(void) {
    return true;
}

/**
 * @brief  get information matrix for IMU pre-integration edge
 * @param  void
 * @return true if success false otherwise
 */
bool IMUPreIntegrator::GetInformation(void) {
    return true;
}

/**
 * @brief  reset pre-integrator state using IMU measurements
 * @param  void
 * @return void
 */
void IMUPreIntegrator::ResetState(const IMUData &init_imu_data) {
    // reset time:
    time_ = init_imu_data.time;

    // a. reset relative translation:
    state_.alpha_ij = Eigen::Vector3d::Zero();
    // b. reset relative velocity:
    state_.beta_ij = Eigen::Vector3d::Zero();
    // c. reset relative orientation:
    state_.theta_ij = Sophus::SO3d();
    // d. set init bias, acceleometer:
    state_.b_a_i = Eigen::Vector3d(
        init_imu_data.accel_bias.x,
        init_imu_data.accel_bias.y,
        init_imu_data.accel_bias.z
    );
    // d. set init bias, gyroscope:
    state_.b_g_i = Eigen::Vector3d(
        init_imu_data.gyro_bias.x,
        init_imu_data.gyro_bias.y,
        init_imu_data.gyro_bias.z
    );

    // reset buffer:
    imu_data_buff_.clear();
    imu_data_buff_.push_back(init_imu_data);
}

/**
 * @brief  update pre-integrator state: mean, covariance and Jacobian
 * @param  void
 * @return void
 */
void IMUPreIntegrator::UpdateState(void) {
    static double T = 0.0;

    static Eigen::Vector3d w_mid = Eigen::Vector3d::Zero();
    static Eigen::Vector3d a_mid = Eigen::Vector3d::Zero();

    static Sophus::SO3d prev_theta_ij = Sophus::SO3d();
    static Sophus::SO3d curr_theta_ij = Sophus::SO3d();
    static Sophus::SO3d d_theta_ij = Sophus::SO3d();

    static Eigen::Matrix3d dR_inv = Eigen::Matrix3d::Zero();
    static Eigen::Matrix3d prev_R = Eigen::Matrix3d::Zero();
    static Eigen::Matrix3d curr_R = Eigen::Matrix3d::Zero();
    static Eigen::Matrix3d prev_R_a_hat = Eigen::Matrix3d::Zero();
    static Eigen::Matrix3d curr_R_a_hat = Eigen::Matrix3d::Zero();

    //
    // parse measurements:
    //
    // get measurement handlers:
    const IMUData &prev_imu_data = imu_data_buff_.at(0);
    const IMUData &curr_imu_data = imu_data_buff_.at(1);

    // get time delta:
    T = curr_imu_data.time - prev_imu_data.time;

    // get measurements:
    const Eigen::Vector3d prev_w(
        prev_imu_data.angular_velocity.x - state_.b_g_i.x(),
        prev_imu_data.angular_velocity.y - state_.b_g_i.y(),
        prev_imu_data.angular_velocity.z - state_.b_g_i.z()
    );
    const Eigen::Vector3d curr_w(
        curr_imu_data.angular_velocity.x - state_.b_g_i.x(),
        curr_imu_data.angular_velocity.y - state_.b_g_i.y(),
        curr_imu_data.angular_velocity.z - state_.b_g_i.z()
    );

    const Eigen::Vector3d prev_a(
        prev_imu_data.linear_acceleration.x - state_.b_a_i.x(),
        prev_imu_data.linear_acceleration.y - state_.b_a_i.y(),
        prev_imu_data.linear_acceleration.z - state_.b_a_i.z()
    );
    const Eigen::Vector3d curr_a(
        curr_imu_data.linear_acceleration.x - state_.b_a_i.x(),
        curr_imu_data.linear_acceleration.y - state_.b_a_i.y(),
        curr_imu_data.linear_acceleration.z - state_.b_a_i.z()
    );

    //
    // a. update mean:
    //
    // 1. get w_mid:
    w_mid = 0.5 * ( prev_w + curr_w );
    // 2. update relative orientation, so3:
    prev_theta_ij = state_.theta_ij;
    d_theta_ij = Sophus::SO3d::exp(w_mid * T);
    state_.theta_ij = state_.theta_ij * d_theta_ij;
    curr_theta_ij = state_.theta_ij;
    // 3. get a_mid:
    a_mid = 0.5 * ( prev_theta_ij * prev_a + curr_theta_ij * curr_a );
    // 4. update relative translation:
    state_.alpha_ij += (state_.beta_ij + 0.5 * a_mid * T) * T;
    // 5. update relative velocity:
    state_.beta_ij += a_mid * T;

    //
    // b. update covariance:
    //
    // 1. intermediate results:
    dR_inv = d_theta_ij.inverse().matrix();
    prev_R = prev_theta_ij.matrix();
    curr_R = curr_theta_ij.matrix();
    prev_R_a_hat = prev_R * Sophus::SO3d::hat(prev_a);
    curr_R_a_hat = curr_R * Sophus::SO3d::hat(curr_a);

    //
    // 2. set up F:
    //
    // F12 & F32:
    F_.block<3, 3>(INDEX_ALPHA, INDEX_THETA) = F_.block<3, 3>(INDEX_BETA, INDEX_THETA) = -0.50 * (prev_R_a_hat + curr_R_a_hat * dR_inv);
    F_.block<3, 3>(INDEX_ALPHA, INDEX_THETA) = 0.50 * T * F_.block<3, 3>(INDEX_ALPHA, INDEX_THETA);
    // F14 & F34:
    F_.block<3, 3>(INDEX_ALPHA,   INDEX_B_A) = F_.block<3, 3>(INDEX_BETA,   INDEX_B_A) = -0.50 * (prev_R + curr_R);
    F_.block<3, 3>(INDEX_ALPHA,   INDEX_B_A) = 0.50 * T * F_.block<3, 3>(INDEX_ALPHA,   INDEX_B_A);
    // F15 & F35:
    F_.block<3, 3>(INDEX_ALPHA,   INDEX_B_G) = F_.block<3, 3>(INDEX_BETA,   INDEX_B_G) = +0.50 * T * curr_R_a_hat;
    F_.block<3, 3>(INDEX_ALPHA,   INDEX_B_G) = 0.50 * T * F_.block<3, 3>(INDEX_ALPHA,   INDEX_B_G);
    // F22:
    F_.block<3, 3>(INDEX_THETA, INDEX_THETA) = -Sophus::SO3d::hat(w_mid);

    //
    // 3. set up G:
    //
    // G11 & G31:
    G_.block<3, 3>(INDEX_ALPHA, INDEX_ALPHA) = G_.block<3, 3>(INDEX_BETA, INDEX_ALPHA) = +0.50 * T * prev_R;
    G_.block<3, 3>(INDEX_ALPHA, INDEX_ALPHA) = 0.50 * T * G_.block<3, 3>(INDEX_ALPHA, INDEX_ALPHA);
    // G12 & G32:
    G_.block<3, 3>(INDEX_ALPHA, INDEX_THETA) = G_.block<3, 3>(INDEX_BETA, INDEX_THETA) = -0.25 * T * curr_R_a_hat;
    G_.block<3, 3>(INDEX_ALPHA, INDEX_THETA) = 0.50 * T * G_.block<3, 3>(INDEX_ALPHA, INDEX_THETA);
    // G13 & G33:
    G_.block<3, 3>(INDEX_ALPHA,  INDEX_BETA) = G_.block<3, 3>(INDEX_BETA,  INDEX_BETA) = 0.5 * curr_R;
    G_.block<3, 3>(INDEX_ALPHA,  INDEX_BETA) = 0.50 * T * G_.block<3, 3>(INDEX_ALPHA,  INDEX_BETA);
    // G14 & G34:
    G_.block<3, 3>(INDEX_ALPHA,   INDEX_B_A) = G_.block<3, 3>(INDEX_BETA,   INDEX_B_A) = -0.25 * T * curr_R_a_hat;
    G_.block<3, 3>(INDEX_ALPHA,   INDEX_B_A) = 0.50 * T * G_.block<3, 3>(INDEX_ALPHA,   INDEX_B_A);

    // 4. update P_:
    MatrixF F = MatrixF::Identity() + T * F_;
    MatrixG G = T * G_;

    P_ = F*P_*F.transpose() + G*R_*G.transpose();

    // 
    // c. update Jacobian:
    //

}

} // namespace lidar_localization