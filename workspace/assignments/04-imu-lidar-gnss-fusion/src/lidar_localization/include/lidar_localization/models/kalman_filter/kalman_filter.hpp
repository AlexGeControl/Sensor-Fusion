/*
 * @Description: IMU-lidar-GNSS fusion using Kalman filter for localization
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */

#ifndef LIDAR_LOCALIZATION_MODELS_KALMAN_FILTER_HPP_
#define LIDAR_LOCALIZATION_MODELS_KALMAN_FILTER_HPP_

#include <yaml-cpp/yaml.h>

#include <deque>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "lidar_localization/sensor_data/imu_data.hpp"

namespace lidar_localization {

class KalmanFilter {
public:
    // dimensions:
    static const int DIM_STATE = 15;
    static const int DIM_PROCESS_NOISE = 6;
    static const int DIM_MEASUREMENT = 6;
    static const int DIM_MEASUREMENT_NOISE = 6;

    // indices:
    static const int INDEX_ERROR_POS = 0;
    static const int INDEX_ERROR_VEL = 3;
    static const int INDEX_ERROR_ORI = 6;
    static const int INDEX_ERROR_GYRO = 9;
    static const int INDEX_ERROR_ACCEL = 12;
    
    // state:
    typedef Eigen::Matrix<double,             DIM_STATE,                     1> VectorX;
    typedef Eigen::Matrix<double,             DIM_STATE,             DIM_STATE> MatrixP;
    // process equation:
    typedef Eigen::Matrix<double,             DIM_STATE,             DIM_STATE> MatrixF;
    typedef Eigen::Matrix<double,             DIM_STATE,     DIM_PROCESS_NOISE> MatrixB;
    typedef Eigen::Matrix<double,     DIM_PROCESS_NOISE,     DIM_PROCESS_NOISE> MatrixQ;
    // measurement equation:
    typedef Eigen::Matrix<double,       DIM_MEASUREMENT,             DIM_STATE> MatrixG;
    typedef Eigen::Matrix<double,       DIM_MEASUREMENT, DIM_MEASUREMENT_NOISE> MatrixC;
    typedef Eigen::Matrix<double, DIM_MEASUREMENT_NOISE, DIM_MEASUREMENT_NOISE> MatrixR;
    // measurement:
    typedef Eigen::Matrix<double,       DIM_MEASUREMENT,                     1> VectorY;
    // Kalman gain:
    typedef Eigen::Matrix<double,             DIM_STATE,       DIM_MEASUREMENT> MatrixK;

    KalmanFilter(const YAML::Node& node);

    /**
     * @brief  init filter
     * @param  imu_data, input IMU measurements
     * @return true if success false otherwise
     */
    void Init(
        const Eigen::Matrix4d &pose,
        const Eigen::Vector3d &vel,
        const IMUData &imu_data
    );

    /**
     * @brief  Kalman update
     * @param  imu_data, input IMU measurements
     * @return true if success false otherwise
     */
    bool Update(const IMUData &imu_data);

    /**
     * @brief  Kalman correction
     * @param  T_nb, odometry delta from lidar frontend
     * @return void
     */
    bool Correct(const IMUData &imu_data, const double &time, const Eigen::Matrix4f &T_nb_lidar);

    /**
     * @brief  get filter time
     * @return filter time as double
     */
    double GetTime(void) const { return time_; }
    
    /**
     * @brief  get odometry estimation
     * @param  pose, init pose
     * @param  vel, init vel
     * @return void
     */
    void GetOdometry(Eigen::Matrix4f &pose, Eigen::Vector3f &vel);
private:
    /**
     * @brief  get unbiased angular velocity in body frame
     * @param  angular_vel, angular velocity measurement
     * @return unbiased angular velocity in body frame
     */
    Eigen::Vector3d GetUnbiasedAngularVel(
        const Eigen::Vector3d &angular_vel
    );
    /**
     * @brief  get unbiased linear acceleration in navigation frame
     * @param  linear_acc, linear acceleration measurement
     * @param  R, corresponding orientation of measurement
     * @return unbiased linear acceleration in navigation frame
     */
    Eigen::Vector3d GetUnbiasedLinearAcc(
        const Eigen::Vector3d &linear_acc,
        const Eigen::Matrix3d &R
    );
    /**
     * @brief  get angular delta
     * @param  index_curr, current imu measurement buffer index
     * @param  index_prev, previous imu measurement buffer index
     * @param  angular_delta, angular delta output
     * @return true if success false otherwise
     */
    bool GetAngularDelta(
        const size_t index_curr, const size_t index_prev,
        Eigen::Vector3d &angular_delta
    );
    /**
     * @brief  get velocity delta
     * @param  index_curr, current imu measurement buffer index
     * @param  index_prev, previous imu measurement buffer index
     * @param  R_curr, corresponding orientation of current imu measurement
     * @param  R_prev, corresponding orientation of previous imu measurement
     * @param  velocity_delta, velocity delta output
     * @return true if success false otherwise
     */
    bool GetVelocityDelta(
        const size_t index_curr, const size_t index_prev,
        const Eigen::Matrix3d &R_curr, const Eigen::Matrix3d &R_prev, 
        double &T, Eigen::Vector3d &velocity_delta
    );
    /**
     * @brief  update orientation with effective rotation angular_delta
     * @param  angular_delta, effective rotation
     * @param  R_curr, current orientation
     * @param  R_prev, previous orientation
     * @return void
     */
    void UpdateOrientation(
        const Eigen::Vector3d &angular_delta,
        Eigen::Matrix3d &R_curr, Eigen::Matrix3d &R_prev
    );
    /**
     * @brief  update orientation with effective velocity change velocity_delta
     * @param  velocity_delta, effective velocity change
     * @return void
     */
    void UpdatePosition(const double &T, const Eigen::Vector3d &velocity_delta);
    /**
     * @brief  update IMU odometry estimation
     * @param  void
     * @return void
     */
    void UpdateOdomEstimation(void);

    /**
     * @brief  get process input, C_nb & f_n, from IMU measurement
     * @param  imu_data, input IMU measurement
     * @param  T, output time delta
     * @param  C_nb, output rotation matrix, body frame -> navigation frame
     * @param  f_n, output accel measurement in navigation frame
     * @return void
     */
    void GetProcessInput(
        const IMUData &imu_data,
        double &T, Eigen::Matrix3d &C_nb, Eigen::Vector3d &f_n
    );
    /**
     * @brief  set process equation
     * @param  C_nb, rotation matrix, body frame -> navigation frame
     * @param  f_n, accel measurement in navigation frame
     * @return void
     */
    void SetProcessEquation(
        const Eigen::Matrix3d &C_nb, const Eigen::Vector3d &f_n
    );
    /**
     * @brief  update process equation
     * @param  imu_data, input IMU measurement
     * @param  T, output time delta
     * @return void
     */
    void UpdateProcessEquation(
        const IMUData &imu_data, double &T
    );
    /**
     * @brief  update error estimation
     * @param  imu_data, input IMU measurement
     * @return void
     */
    void UpdateErrorEstimation(const IMUData &imu_data);

    // data buff:
    std::deque<IMUData> imu_data_buff_;

    // time:
    double time_;

    // odometry estimation from IMU integration:
    Eigen::Matrix4d pose_ = Eigen::Matrix4d::Identity();
    Eigen::Vector3d vel_ = Eigen::Vector3d::Zero();

    // state:
    VectorX X_;
    MatrixP P_ = MatrixP::Zero();
    // process & measurement equations:
    MatrixF F_ = MatrixF::Zero();
    MatrixB B_ = MatrixB::Zero();
    MatrixQ Q_ = MatrixQ::Zero();

    MatrixG G_ = MatrixG::Zero();
    MatrixC C_ = MatrixC::Zero();
    MatrixR R_ = MatrixR::Zero();
    // measurement:
    VectorY Y_;

    // earth constants:
    Eigen::Vector3d g_;
    Eigen::Vector3d w_;

    // hyper-params:
    // a. earth constants:
    struct {
        double GRAVITY_MAGNITUDE;
        double ROTATION_SPEED;
        double LATITUDE;
    } EARTH;
    // b. prior state covariance, process & measurement noise:
    struct {
        struct {
            double POS;
            double VEL;
            double ORIENTATION;
            double EPSILON;
            double DELTA;
        } PRIOR;
        struct {
            double GYRO;
            double ACCEL;
        } PROCESS;
        struct {
            double POS;
            double ORIENTATION;
        } MEASUREMENT;
    } COV;
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_MODELS_KALMAN_FILTER_HPP_