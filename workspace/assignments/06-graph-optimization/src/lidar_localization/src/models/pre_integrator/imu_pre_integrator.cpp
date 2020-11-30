/*
 * @Description: IMU pre-integrator for LIO mapping, implementation
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 */

#include "lidar_localization/models/pre_integrator/imu_pre_integrator.hpp"

namespace lidar_localization {

IMUPreIntegrator::IMUPreIntegrator(void) {
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
    state_.q_ij = Eigen::Quaterniond::Identity();
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
 * @brief  update pre-integrator state, mean
 * @param  void
 * @return void
 */
void IMUPreIntegrator::UpdateStateMean(void) {
    // get measurement handlers:
    const IMUData &prev_imu_data = imu_data_buff_.at(0);
    const IMUData &curr_imu_data = imu_data_buff_.at(1);
    
    // parse orientations:
    const Eigen::Matrix3d prev_C_nb = prev_imu_data.GetOrientationMatrix().cast<double>();
    const Eigen::Matrix3d curr_C_nb = curr_imu_data.GetOrientationMatrix().cast<double>();

    //
    // calculate mid values:
    //
    // a. mid-value linear acceleration:
    const Eigen::Vector3d prev_a(
        prev_imu_data.linear_acceleration.x,
        prev_imu_data.linear_acceleration.y,
        prev_imu_data.linear_acceleration.z
    );
    const Eigen::Vector3d curr_a(
        curr_imu_data.linear_acceleration.x,
        curr_imu_data.linear_acceleration.y,
        curr_imu_data.linear_acceleration.z
    );
    Eigen::Vector3d a_mid = 0.5 * (
        prev_C_nb * (prev_a - state_.b_a_i) + curr_C_nb * (curr_a - state_.b_a_i)
    );

    // b. mid-value angular velocity:
    const Eigen::Vector3d prev_w(
        prev_imu_data.angular_velocity.x,
        prev_imu_data.angular_velocity.y,
        prev_imu_data.angular_velocity.z
    );
    const Eigen::Vector3d curr_w(
        curr_imu_data.angular_velocity.x,
        curr_imu_data.angular_velocity.y,
        curr_imu_data.angular_velocity.z
    );
    Eigen::Vector3d w_mid = 0.5 * ( prev_w + curr_w );

    //
    // update state, mean
    //
    double T = curr_imu_data.time - prev_imu_data.time;
    // a. update relative translation:
    state_.alpha_ij += state_.beta_ij * T + 0.5 * a_mid * T * T;
    // b. update relative velocity:
    state_.beta_ij += a_mid * T;
    // c. update relative orientation:
    state_.q_ij *= Eigen::Quaterniond(
        1.0, 0.5*w_mid.x(), 0.5*w_mid.y(), 0.5*w_mid.z()
    );
    state_.q_ij.normalize();
}

/**
 * @brief  update pre-integrator state, covariance
 * @param  void
 * @return void
 */
void IMUPreIntegrator::UpdateStateCovariance(void) {
    
}

} // namespace lidar_localization