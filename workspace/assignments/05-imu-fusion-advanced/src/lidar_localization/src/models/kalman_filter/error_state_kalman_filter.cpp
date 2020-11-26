/*
 * @Description: Error-State Kalman Filter for IMU-Lidar-GNSS-Odo fusion
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */
#include <limits>

#include <cmath>
#include <iostream>
#include <fstream>
#include <ostream>

// use sophus to handle so3 hat & SO3 log operations:
#include <sophus/so3.hpp>

// SVD for observability analysis:
#include <Eigen/SVD>

#include "lidar_localization/models/kalman_filter/error_state_kalman_filter.hpp"

#include "lidar_localization/global_defination/global_defination.h"

#include "lidar_localization/tools/CSVWriter.hpp"

#include "glog/logging.h"

namespace lidar_localization {

ErrorStateKalmanFilter::ErrorStateKalmanFilter(const YAML::Node& node) {
    //
    // parse config:
    // 
    // a. earth constants:
    EARTH.GRAVITY_MAGNITUDE = node["earth"]["gravity_magnitude"].as<double>();
    EARTH.ROTATION_SPEED = node["earth"]["rotation_speed"].as<double>();
    EARTH.LATITUDE = node["earth"]["latitude"].as<double>();
    EARTH.LATITUDE *= M_PI / 180.0;
    // b. prior state covariance:
    COV.PRIOR.POSI = node["covariance"]["prior"]["pos"].as<double>();
    COV.PRIOR.VEL = node["covariance"]["prior"]["vel"].as<double>();
    COV.PRIOR.ORI = node["covariance"]["prior"]["ori"].as<double>();
    COV.PRIOR.EPSILON = node["covariance"]["prior"]["epsilon"].as<double>();
    COV.PRIOR.DELTA = node["covariance"]["prior"]["delta"].as<double>();
    // c. process noise:
    COV.PROCESS.GYRO = node["covariance"]["process"]["gyro"].as<double>();
    COV.PROCESS.ACCEL = node["covariance"]["process"]["accel"].as<double>();
    // d. measurement noise:
    COV.MEASUREMENT.POSI = node["covariance"]["measurement"]["pos"].as<double>();
    COV.MEASUREMENT.VEL = node["covariance"]["measurement"]["vel"].as<double>();
    COV.MEASUREMENT.ORI = node["covariance"]["measurement"]["ori"].as<double>();

    // prompt:
    LOG(INFO) << std::endl 
              << "Error-State Kalman Filter params:" << std::endl
              << "\tgravity magnitude: " << EARTH.GRAVITY_MAGNITUDE << std::endl
              << "\tearth rotation speed: " << EARTH.ROTATION_SPEED << std::endl
              << "\tlatitude: " << EARTH.LATITUDE << std::endl
              << std::endl
              << "\tprior cov. pos.: " << COV.PRIOR.POSI  << std::endl
              << "\tprior cov. vel.: " << COV.PRIOR.VEL << std::endl
              << "\tprior cov. ori: " << COV.PRIOR.ORI << std::endl
              << "\tprior cov. epsilon.: " << COV.PRIOR.EPSILON  << std::endl
              << "\tprior cov. delta.: " << COV.PRIOR.DELTA << std::endl
              << std::endl
              << "\tprocess noise gyro.: " << COV.PROCESS.GYRO << std::endl
              << "\tprocess noise accel.: " << COV.PROCESS.ACCEL << std::endl
              << std::endl
              << "\tmeasurement noise pos.: " << COV.MEASUREMENT.POSI << std::endl
              << "\tmeasurement noise vel.: " << COV.MEASUREMENT.VEL << std::endl
              << "\tmeasurement noise orientation.: " << COV.MEASUREMENT.ORI << std::endl
              << std::endl;
    
    //
    // init filter:
    //
    // a. earth constants:
    g_ = Eigen::Vector3d(
        0.0, 
        0.0, 
        EARTH.GRAVITY_MAGNITUDE
    );
    w_ = Eigen::Vector3d(
        0.0,
        EARTH.ROTATION_SPEED*cos(EARTH.LATITUDE),
        EARTH.ROTATION_SPEED*sin(EARTH.LATITUDE)
    );
    // b. prior state & covariance:
    ResetState();
    ResetCovariance();
    // c. process noise:
    Q_.block<3, 3>(0, 0) = COV.PROCESS.GYRO*Eigen::Matrix3d::Identity();
    Q_.block<3, 3>(3, 3) = COV.PROCESS.ACCEL*Eigen::Matrix3d::Identity();

    // d. measurement noise:
    RPose_.block<3, 3>(0, 0) = COV.MEASUREMENT.POSI*Eigen::Matrix3d::Identity();
    RPose_.block<3, 3>(3, 3) = COV.MEASUREMENT.ORI*Eigen::Matrix3d::Identity();

    RPosi_.block<3, 3>(0, 0) = COV.MEASUREMENT.POSI*Eigen::Matrix3d::Identity();

    RPosiVel_.block<3, 3>(0, 0) = COV.MEASUREMENT.POSI*Eigen::Matrix3d::Identity();
    RPosiVel_.block<3, 3>(3, 3) = COV.MEASUREMENT.VEL*Eigen::Matrix3d::Identity();
    // RPosiVel_.block<3, 3>(6, 6) = COV.MEASUREMENT.ORI*Eigen::Matrix3d::Identity();

    // e. process equation:
    F_.block<3, 3>(  INDEX_ERROR_POS,   INDEX_ERROR_VEL) = Eigen::Matrix3d::Identity();
    F_.block<3, 3>(  INDEX_ERROR_ORI,   INDEX_ERROR_ORI) = Sophus::SO3d::hat(-w_).matrix();

    // f. measurement equation:
    GPose_.block<3, 3>(0, INDEX_ERROR_POS) = Eigen::Matrix3d::Identity();
    GPose_.block<3, 3>(3, INDEX_ERROR_ORI) = Eigen::Matrix3d::Identity();
    CPose_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    CPose_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();

    GPosi_.block<3, 3>(0, INDEX_ERROR_POS) = Eigen::Matrix3d::Identity();
    CPosi_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

    GPosiVel_.block<3, 3>(0, INDEX_ERROR_POS) = Eigen::Matrix3d::Identity();
    // GPosiVel_.block<3, 3>(6, INDEX_ERROR_ORI) = Eigen::Matrix3d::Identity();
    CPosiVel_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    CPosiVel_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
    // CPosiVel_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();

    // init soms:
    SOMPose_.block<DIM_MEASUREMENT_POSE, DIM_STATE>(0, 0) = GPose_;
    SOMPosi_.block<DIM_MEASUREMENT_POSI, DIM_STATE>(0, 0) = GPosi_;
    SOMPosiVel_.block<DIM_MEASUREMENT_POSI_VEL, DIM_STATE>(0, 0) = GPosiVel_;
}

/**
 * @brief  init filter
 * @param  pose, init pose
 * @param  vel, init vel
 * @param  imu_data, init IMU measurements
 * @return true if success false otherwise
 */
void ErrorStateKalmanFilter::Init(
    const Eigen::Vector3d &vel,
    const IMUData &imu_data
) {
    // init odometry:
    Eigen::Matrix3d C_nb = imu_data.GetOrientationMatrix().cast<double>();
    // a. init C_nb using IMU estimation: 
    pose_.block<3, 3>(0, 0) = C_nb;
    // b. convert flu velocity into navigation frame:
    vel_ = C_nb*vel;

    // save init pose:
    init_pose_ = pose_;

    // init IMU data buffer:
    imu_data_buff_.clear();
    imu_data_buff_.push_back(imu_data);

    // init filter time:
    time_ = imu_data.time;

    // set process equation in case of one step prediction & correction:
    Eigen::Vector3d linear_acc_init(
        imu_data.linear_acceleration.x,
        imu_data.linear_acceleration.y,
        imu_data.linear_acceleration.z
    );
    // covert to navigation frame:
    linear_acc_init = C_nb * (linear_acc_init - accl_bias_);

    // init process equation, in case of direct correct step:
    UpdateProcessEquation(linear_acc_init);

    LOG(INFO) << std::endl 
              << "Kalman Filter Inited at " << static_cast<int>(time_) << std::endl
              << "Init Position: " 
              << pose_(0, 3) << ", "
              << pose_(1, 3) << ", "
              << pose_(2, 3) << std::endl
              << "Init Velocity: "
              << vel_.x() << ", "
              << vel_.y() << ", "
              << vel_.z() << std::endl;
}

/**
 * @brief  Kalman update
 * @param  imu_data, input IMU measurements
 * @return true if success false otherwise
 */
bool ErrorStateKalmanFilter::Update(const IMUData &imu_data) {
    // update IMU buff:
    if (time_ < imu_data.time) {
        // update buffer:
        imu_data_buff_.push_back(imu_data);

        // update IMU odometry:
        Eigen::Vector3d linear_acc_mid;
        UpdateOdomEstimation(linear_acc_mid);
        
        // update error estimation:
        double T = imu_data.time - time_;
        UpdateErrorEstimation(T, linear_acc_mid);
        
        // move forward:
        imu_data_buff_.pop_front();

        // update filter time:
        time_ = imu_data.time;

        return true;
    }

    return false;
}

/**
 * @brief  Kalman correction, pose measurement and other measurement in body frame
 * @param  measurement_type, input measurement type
 * @param  measurement, input measurement
 * @return void                                   
 */
bool ErrorStateKalmanFilter::Correct(
    const IMUData &imu_data, 
    const MeasurementType &measurement_type, const Measurement &measurement
) { 
    static Measurement measurement_;

    // get time delta:
    double time_delta = measurement.time - time_;

    if ( time_delta > -0.05 ) {
        // perform Kalman prediction:
        if ( time_ < measurement.time ) {
            Update(imu_data);
        }

        // get observation in navigation frame:
        measurement_ = measurement;
        measurement_.T_nb = init_pose_ * measurement_.T_nb;

        // correct error estimation:
        CorrectErrorEstimation(measurement_type, measurement);

        // eliminate error:
        EliminateError();

        // reset error state:
        ResetState();

        return true;
    }

    LOG(INFO) << "Kalman Correct: Observation is not synced with filter. Skip, " 
              << (int)measurement.time << " <-- " << (int)time_ << " @ " << time_delta
              << std::endl; 
    
    return false;
}

/**
 * @brief  get odometry estimation
 * @param  pose, init pose
 * @param  vel, init vel
 * @return void
 */
void ErrorStateKalmanFilter::GetOdometry(
    Eigen::Matrix4f &pose, Eigen::Vector3f &vel
) {
    // init:
    Eigen::Matrix4d pose_double = pose_;
    Eigen::Vector3d vel_double = vel_;

    // eliminate error:
    // a. position:
    pose_double.block<3, 1>(0, 3) = pose_double.block<3, 1>(0, 3) - X_.block<3, 1>(INDEX_ERROR_POS, 0);
    // b. velocity:
    vel_double = vel_double - X_.block<3, 1>(INDEX_ERROR_VEL, 0);
    // apply motion constraint:
    ApplyMotionConstraint();
    // c. orientation:
    Eigen::Matrix3d C_nn = Sophus::SO3d::exp(X_.block<3, 1>(INDEX_ERROR_ORI, 0)).matrix();
    pose_double.block<3, 3>(0, 0) = C_nn*pose_double.block<3, 3>(0, 0);

    // finally:
    pose_double = init_pose_.inverse() * pose_double;
    vel_double = init_pose_.block<3, 3>(0, 0).transpose() * vel_double;

    pose = pose_double.cast<float>();
    vel = vel_double.cast<float>();
}

/**
 * @brief  get covariance estimation
 * @param  cov, covariance output
 * @return void
 */
void ErrorStateKalmanFilter::GetCovariance(Cov &cov) {
    static int OFFSET_X = 0;
    static int OFFSET_Y = 1;
    static int OFFSET_Z = 2;

    // a. delta position:
    cov.pos.x = P_(INDEX_ERROR_POS + OFFSET_X, INDEX_ERROR_POS + OFFSET_X);
    cov.pos.y = P_(INDEX_ERROR_POS + OFFSET_Y, INDEX_ERROR_POS + OFFSET_Y);
    cov.pos.z = P_(INDEX_ERROR_POS + OFFSET_Z, INDEX_ERROR_POS + OFFSET_Z);

    // b. delta velocity:
    cov.vel.x = P_(INDEX_ERROR_VEL + OFFSET_X, INDEX_ERROR_VEL + OFFSET_X);
    cov.vel.y = P_(INDEX_ERROR_VEL + OFFSET_Y, INDEX_ERROR_VEL + OFFSET_Y);
    cov.vel.z = P_(INDEX_ERROR_VEL + OFFSET_Z, INDEX_ERROR_VEL + OFFSET_Z);

    // c. delta orientation:
    cov.ori.x = P_(INDEX_ERROR_ORI + OFFSET_X, INDEX_ERROR_ORI + OFFSET_X);
    cov.ori.y = P_(INDEX_ERROR_ORI + OFFSET_Y, INDEX_ERROR_ORI + OFFSET_Y);
    cov.ori.z = P_(INDEX_ERROR_ORI + OFFSET_Z, INDEX_ERROR_ORI + OFFSET_Z);

    // d. gyro. bias:
    cov.gyro_bias.x = P_(INDEX_ERROR_GYRO + OFFSET_X, INDEX_ERROR_GYRO + OFFSET_X);
    cov.gyro_bias.y = P_(INDEX_ERROR_GYRO + OFFSET_Y, INDEX_ERROR_GYRO + OFFSET_Y);
    cov.gyro_bias.z = P_(INDEX_ERROR_GYRO + OFFSET_Z, INDEX_ERROR_GYRO + OFFSET_Z);

    // e. accel bias:
    cov.accel_bias.x = P_(INDEX_ERROR_ACCEL + OFFSET_X, INDEX_ERROR_ACCEL + OFFSET_X);
    cov.accel_bias.y = P_(INDEX_ERROR_ACCEL + OFFSET_Y, INDEX_ERROR_ACCEL + OFFSET_Y);
    cov.accel_bias.z = P_(INDEX_ERROR_ACCEL + OFFSET_Z, INDEX_ERROR_ACCEL + OFFSET_Z);
}

/**
 * @brief  get unbiased angular velocity in body frame
 * @param  angular_vel, angular velocity measurement
 * @param  R, corresponding orientation of measurement
 * @return unbiased angular velocity in body frame
 */
inline Eigen::Vector3d ErrorStateKalmanFilter::GetUnbiasedAngularVel(
    const Eigen::Vector3d &angular_vel,
    const Eigen::Matrix3d &R
) {
    return angular_vel - gyro_bias_ - R.transpose() * w_;
}

/**
 * @brief  get unbiased linear acceleration in navigation frame
 * @param  linear_acc, linear acceleration measurement
 * @param  R, corresponding orientation of measurement
 * @return unbiased linear acceleration in navigation frame
 */
inline Eigen::Vector3d ErrorStateKalmanFilter::GetUnbiasedLinearAcc(
    const Eigen::Vector3d &linear_acc,
    const Eigen::Matrix3d &R
) {
    return R*(linear_acc - accl_bias_) - g_;
}

/**
 * @brief  apply motion constraint on velocity estimation
 * @param  void
 * @return void
 */
void ErrorStateKalmanFilter::ApplyMotionConstraint(void) {
    const Eigen::Matrix3d C_nb = pose_.block<3, 3>(0, 0);

    Eigen::Vector3d v_b = C_nb.transpose() * vel_;
    v_b.y() = v_b.z() = 0.0;
    vel_ = C_nb * v_b;
}

/**
 * @brief  get angular delta
 * @param  index_curr, current imu measurement buffer index
 * @param  index_prev, previous imu measurement buffer index
 * @param  angular_delta, angular delta output
 * @return true if success false otherwise
 */
bool ErrorStateKalmanFilter::GetAngularDelta(
    const size_t index_curr, const size_t index_prev,
    Eigen::Vector3d &angular_delta
) {
    if (
        index_curr <= index_prev ||
        imu_data_buff_.size() <= index_curr
    ) {
        return false;
    }

    const IMUData &imu_data_curr = imu_data_buff_.at(index_curr);
    const IMUData &imu_data_prev = imu_data_buff_.at(index_prev);

    double delta_t = imu_data_curr.time - imu_data_prev.time;

    Eigen::Vector3d angular_vel_curr = Eigen::Vector3d(
        imu_data_curr.angular_velocity.x,
        imu_data_curr.angular_velocity.y,
        imu_data_curr.angular_velocity.z
    );
    Eigen::Matrix3d R_curr = imu_data_curr.GetOrientationMatrix().cast<double>();
    angular_vel_curr = GetUnbiasedAngularVel(angular_vel_curr, R_curr);

    Eigen::Vector3d angular_vel_prev = Eigen::Vector3d(
        imu_data_prev.angular_velocity.x,
        imu_data_prev.angular_velocity.y,
        imu_data_prev.angular_velocity.z
    );
    Eigen::Matrix3d R_prev = imu_data_prev.GetOrientationMatrix().cast<double>();
    angular_vel_prev = GetUnbiasedAngularVel(angular_vel_prev, R_prev);

    angular_delta = 0.5*delta_t*(angular_vel_curr + angular_vel_prev);

    return true;
}

/**
 * @brief  update orientation with effective rotation angular_delta
 * @param  angular_delta, effective rotation
 * @param  R_curr, current orientation
 * @param  R_prev, previous orientation
 * @return void
 */
void ErrorStateKalmanFilter::UpdateOrientation(
    const Eigen::Vector3d &angular_delta,
    Eigen::Matrix3d &R_curr, Eigen::Matrix3d &R_prev
) {
    // magnitude:
    double angular_delta_mag = angular_delta.norm();
    // direction:
    Eigen::Vector3d angular_delta_dir = angular_delta.normalized();

    // build delta q:
    double angular_delta_cos = cos(angular_delta_mag/2.0);
    double angular_delta_sin = sin(angular_delta_mag/2.0);
    Eigen::Quaterniond dq(
        angular_delta_cos, 
        angular_delta_sin*angular_delta_dir.x(), 
        angular_delta_sin*angular_delta_dir.y(), 
        angular_delta_sin*angular_delta_dir.z()
    );
    Eigen::Quaterniond q(pose_.block<3, 3>(0, 0));
    
    // update:
    q = q*dq;
    
    // write back:
    R_prev = pose_.block<3, 3>(0, 0);
    pose_.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    R_curr = pose_.block<3, 3>(0, 0);
}

/**
 * @brief  get velocity delta
 * @param  index_curr, current imu measurement buffer index
 * @param  index_prev, previous imu measurement buffer index
 * @param  R_curr, corresponding orientation of current imu measurement
 * @param  R_prev, corresponding orientation of previous imu measurement
 * @param  velocity_delta, velocity delta output
 * @param  linear_acc_mid, mid-value unbiased linear acc
 * @return true if success false otherwise
 */
bool ErrorStateKalmanFilter::GetVelocityDelta(
    const size_t index_curr, const size_t index_prev,
    const Eigen::Matrix3d &R_curr, const Eigen::Matrix3d &R_prev, 
    double &T, 
    Eigen::Vector3d &velocity_delta, 
    Eigen::Vector3d &linear_acc_mid
) {
    if (
        index_curr <= index_prev ||
        imu_data_buff_.size() <= index_curr
    ) {
        return false;
    }

    const IMUData &imu_data_curr = imu_data_buff_.at(index_curr);
    const IMUData &imu_data_prev = imu_data_buff_.at(index_prev);

    T = imu_data_curr.time - imu_data_prev.time;

    Eigen::Vector3d linear_acc_curr = Eigen::Vector3d(
        imu_data_curr.linear_acceleration.x,
        imu_data_curr.linear_acceleration.y,
        imu_data_curr.linear_acceleration.z
    );
    linear_acc_curr = GetUnbiasedLinearAcc(linear_acc_curr, R_curr);
    Eigen::Vector3d linear_acc_prev = Eigen::Vector3d(
        imu_data_prev.linear_acceleration.x,
        imu_data_prev.linear_acceleration.y,
        imu_data_prev.linear_acceleration.z
    );
    linear_acc_prev = GetUnbiasedLinearAcc(linear_acc_prev, R_prev);
    
    // mid-value acc can improve error state prediction accuracy:
    linear_acc_mid = 0.5*(linear_acc_curr + linear_acc_prev);
    velocity_delta = T*linear_acc_mid;

    return true;
}

/**
 * @brief  update orientation with effective velocity change velocity_delta
 * @param  T, timestamp delta 
 * @param  velocity_delta, effective velocity change
 * @return void
 */
void ErrorStateKalmanFilter::UpdatePosition(const double &T, const Eigen::Vector3d &velocity_delta) {
    pose_.block<3, 1>(0, 3) += T*vel_ + 0.5*T*velocity_delta;
    vel_ += velocity_delta;
}

/**
 * @brief  update IMU odometry estimation
 * @param  linear_acc_mid, output mid-value unbiased linear acc
 * @return void
 */
void ErrorStateKalmanFilter::UpdateOdomEstimation(Eigen::Vector3d &linear_acc_mid) {
    // get deltas:
    Eigen::Vector3d angular_delta; 
    GetAngularDelta(1, 0, angular_delta);

    // update orientation:
    Eigen::Matrix3d R_curr, R_prev;
    UpdateOrientation(angular_delta, R_curr, R_prev);

    // get velocity delta:
    double T;
    Eigen::Vector3d velocity_delta;
    // save mid-value unbiased linear acc for error-state update:
    GetVelocityDelta(
        1, 0, 
        R_curr, R_prev, 
        T, velocity_delta, linear_acc_mid
    );

    // update position:
    UpdatePosition(T, velocity_delta);

    // apply motion constraint:
    ApplyMotionConstraint();
}

/**
 * @brief  set process equation
 * @param  C_nb, rotation matrix, body frame -> navigation frame
 * @param  f_n, accel measurement in navigation frame
 * @return void
 */
void ErrorStateKalmanFilter::SetProcessEquation(
    const Eigen::Matrix3d &C_nb, const Eigen::Vector3d &f_n
) {
    // a. set process equation for delta vel:
    F_.block<3, 3>(INDEX_ERROR_VEL,  INDEX_ERROR_ORI) = Sophus::SO3d::hat(f_n).matrix();
    F_.block<3, 3>(INDEX_ERROR_VEL,INDEX_ERROR_ACCEL) = B_.block<3, 3>(INDEX_ERROR_VEL, 3) = C_nb;
    // b. set process equation for delta ori:
    // TODO: update the block influenced by earth rotation speed:
    F_.block<3, 3>(INDEX_ERROR_ORI, INDEX_ERROR_GYRO) = B_.block<3, 3>(INDEX_ERROR_ORI, 0) = -C_nb;
}

/**
 * @brief  update process equation
 * @param  imu_data, input IMU measurement
 * @param  T, output time delta
 * @return void
 */
void ErrorStateKalmanFilter::UpdateProcessEquation(
    const Eigen::Vector3d &linear_acc_mid
) { 
    // set linearization point:
    Eigen::Matrix3d C_nb = pose_.block<3, 3>(0, 0);
    Eigen::Vector3d f_n = linear_acc_mid + g_;

    // set process equation:
    SetProcessEquation(C_nb, f_n);
}

/**
 * @brief  update error estimation
 * @param  linear_acc_mid, input mid-value unbiased linear acc
 * @return void
 */
void ErrorStateKalmanFilter::UpdateErrorEstimation(
    const double &T,
    const Eigen::Vector3d &linear_acc_mid
) {
    static int count = 0;

    static MatrixF F_1st;
    static MatrixF F_2nd;

    // update process equation:
    UpdateProcessEquation(linear_acc_mid);

    // get discretized process equations:
    F_1st = T*F_;
    F_2nd = 0.5*T*F_*F_1st;
    // approximate to 2nd order:
    MatrixF F = MatrixF::Identity() + F_1st + F_2nd;
    MatrixB B = T*B_;

    // perform Kalman prediction:
    X_ = F*X_;
    P_ = F*P_*F.transpose() + B*Q_*B.transpose();

    // update counter:
    if (
        0 == (++count % 10) 
    ) {
        count = 0;
        /*
        // covariance monitor, update step, debugging only:
        LOG(INFO) << std::endl 
                  << static_cast<int>(imu_data.time) << " Update" << std::endl
                  << P_( 0,  0) << ", " << P_( 1,  1) << ", " << P_( 2,  2) << std::endl
                  << P_( 3,  3) << ", " << P_( 4,  4) << ", " << P_( 5,  5) << std::endl
                  << P_( 6,  6) << ", " << P_( 7,  7) << ", " << P_( 8,  8) << std::endl
                  << P_( 9,  9) << ", " << P_(10, 10) << ", " << P_(11, 11) << std::endl
                  << P_(12, 12) << ", " << P_(13, 13) << ", " << P_(14, 14) << std::endl
                  << std::endl;
        */
    }
}

/**
 * @brief  correct error estimation using pose measurement
 * @param  T_nb, input pose measurement
 * @return void
 */
void ErrorStateKalmanFilter::CorrectErrorEstimationPose(
    const Eigen::Matrix4d &T_nb
) {
    // parse measurement:
    Eigen::Vector3d P_nn_obs = pose_.block<3, 1>(0,3) - T_nb.block<3, 1>(0,3);
    Eigen::Matrix3d C_nn_obs = pose_.block<3, 3>(0,0) * T_nb.block<3, 3>(0,0).transpose();

    YPose_.block<3, 1>(0, 0) = P_nn_obs;
    YPose_.block<3, 1>(3, 0) = Sophus::SO3d::vee(Eigen::Matrix3d::Identity() - C_nn_obs);

    // build Kalman gain:
    MatrixRPose R = GPose_*P_*GPose_.transpose() + RPose_;
    MatrixKPose K = P_*GPose_.transpose()*R.inverse();

    // perform Kalman correct:
    P_ = (MatrixP::Identity() - K*GPose_)*P_;
    X_ = X_ + K*(YPose_ - GPose_*X_);
}

/**
 * @brief  correct error estimation using position measurement
 * @param  T_nb, input position measurement
 * @return void
 */
void ErrorStateKalmanFilter::CorrectErrorEstimationPosi(
    const Eigen::Matrix4d &T_nb
) {
    // parse measurement:
    Eigen::Vector3d P_nn_obs = pose_.block<3, 1>(0,3) - T_nb.block<3, 1>(0,3);

    YPosi_.block<3, 1>(0, 0) = P_nn_obs;

    // build Kalman gain:
    MatrixRPosi R = GPosi_*P_*GPosi_.transpose() + RPosi_;
    MatrixKPosi K = P_*GPosi_.transpose()*R.inverse();

    // perform Kalman correct:
    P_ = (MatrixP::Identity() - K*GPosi_)*P_;
    X_ = X_ + K*(YPosi_ - GPosi_*X_);
}

/**
 * @brief  correct error estimation using navigation position and body velocity measurement
 * @param  T_nb, input position measurement
 * @param  v_b, input velocity measurement
 * @return void
 */
void ErrorStateKalmanFilter::CorrectErrorEstimationPosiVel(
    const Eigen::Matrix4d &T_nb, const Eigen::Vector3d &v_b
) {
    // parse measurement:
    Eigen::Vector3d P_nn_obs = pose_.block<3, 1>(0,3) - T_nb.block<3, 1>(0,3);
    Eigen::Vector3d v_bb_obs = pose_.block<3, 3>(0,0).transpose()*vel_ - v_b;
    // apply motion constraint:
    v_bb_obs.y() = v_bb_obs.z() = 0.0;
    // Eigen::Matrix3d C_nn_obs = pose_.block<3, 3>(0,0) * T_nb.block<3, 3>(0,0).transpose();

    YPosiVel_.block<3, 1>(0, 0) = P_nn_obs;
    YPosiVel_.block<3, 1>(3, 0) = v_bb_obs ;
    // YPosiVel_.block<3, 1>(6, 0) = Sophus::SO3d::vee(Eigen::Matrix3d::Identity() - C_nn_obs);

    // set measurement equation:
    GPosiVel_.block<3, 3>(3, INDEX_ERROR_VEL) =  pose_.block<3, 3>(0,0).transpose();
    GPosiVel_.block<3, 3>(3, INDEX_ERROR_ORI) = -pose_.block<3, 3>(0,0).transpose()*Sophus::SO3d::hat(vel_);

    // build Kalman gain:
    MatrixRPosiVel R = GPosiVel_*P_*GPosiVel_.transpose() + RPosiVel_;
    MatrixKPosiVel K = P_*GPosiVel_.transpose()*R.inverse();

    // perform Kalman correct:
    P_ = (MatrixP::Identity() - K*GPosiVel_)*P_;
    X_ = X_ + K*(YPosiVel_ - GPosiVel_*X_);
}

/**
 * @brief  correct error estimation
 * @param  measurement_type, measurement type
 * @param  measurement, input measurement
 * @return void
 */
void ErrorStateKalmanFilter::CorrectErrorEstimation(
    const MeasurementType &measurement_type, 
    const Measurement &measurement
) {
    static int count = 0;

    switch ( measurement_type ) {
        case MeasurementType::POSE:
            CorrectErrorEstimationPose(measurement.T_nb);
            break;
        case MeasurementType::POSI:
            CorrectErrorEstimationPosi(measurement.T_nb);
            break;
        case MeasurementType::POSI_VEL:
            CorrectErrorEstimationPosiVel(
                measurement.T_nb,
                measurement.v_b
            );
        default:
            break;
    }

    if (
        0 == (++count % 10) 
    ) {
        count = 0;
        /*
        // covariance monitor, correct step, debugging only
        LOG(INFO) << std::endl 
                  << "Correct" << std::endl
                  << P_( 0,  0) << ", " << P_( 1,  1) << ", " << P_( 2,  2) << std::endl
                  << P_( 3,  3) << ", " << P_( 4,  4) << ", " << P_( 5,  5) << std::endl
                  << P_( 6,  6) << ", " << P_( 7,  7) << ", " << P_( 8,  8) << std::endl
                  << P_( 9,  9) << ", " << P_(10, 10) << ", " << P_(11, 11) << std::endl
                  << P_(12, 12) << ", " << P_(13, 13) << ", " << P_(14, 14) << std::endl
                  << std::endl;
        */
    }
}

/**
 * @brief  eliminate error
 * @param  void
 * @return void
 */
void ErrorStateKalmanFilter::EliminateError(void) {
    // a. position:
    pose_.block<3, 1>(0, 3) = pose_.block<3, 1>(0, 3) - X_.block<3, 1>(INDEX_ERROR_POS, 0);
    // b. velocity:
    vel_ = vel_ - X_.block<3, 1>(INDEX_ERROR_VEL, 0);
    // apply motion constraint:
    ApplyMotionConstraint();
    // c. orientation:
    Eigen::Matrix3d C_nn = Sophus::SO3d::exp(X_.block<3, 1>(INDEX_ERROR_ORI, 0)).matrix();
    pose_.block<3, 3>(0, 0) = C_nn*pose_.block<3, 3>(0, 0);

    // d. gyro bias:
    if ( IsCovStable(INDEX_ERROR_GYRO) ) {
        gyro_bias_ += X_.block<3, 1>(INDEX_ERROR_GYRO, 0);
    }
    
    // e. accel bias:
    if ( IsCovStable(INDEX_ERROR_ACCEL) ) {
        accl_bias_ += X_.block<3, 1>(INDEX_ERROR_ACCEL, 0);
    }
}

/**
 * @brief  is covariance stable
 * @param  INDEX_OFSET, state index offset
 * @param  THRESH, covariance threshold, defaults to 1.0e-5
 * @return void
 */
bool ErrorStateKalmanFilter::IsCovStable(
    const int INDEX_OFSET,
    const double THRESH
) {
    for (int i = 0; i < 3; ++i) {
        if ( P_(INDEX_OFSET + i, INDEX_OFSET + i) > THRESH ) {
            return false;
        }
    }

    return true;
}

/**
 * @brief  reset filter state
 * @param  void
 * @return void
 */
void ErrorStateKalmanFilter::ResetState(void) {
    // reset current state:
    X_ = VectorX::Zero();
}

/**
 * @brief  reset filter covariance
 * @param  void
 * @return void
 */
void ErrorStateKalmanFilter::ResetCovariance(void) {
    P_ = MatrixP::Zero();
    
    P_.block<3, 3>(  INDEX_ERROR_POS,   INDEX_ERROR_POS) = COV.PRIOR.POSI*Eigen::Matrix3d::Identity();
    P_.block<3, 3>(  INDEX_ERROR_VEL,   INDEX_ERROR_VEL) = COV.PRIOR.VEL*Eigen::Matrix3d::Identity();
    P_.block<3, 3>(  INDEX_ERROR_ORI,   INDEX_ERROR_ORI) = COV.PRIOR.ORI*Eigen::Matrix3d::Identity();
    P_.block<3, 3>( INDEX_ERROR_GYRO,  INDEX_ERROR_GYRO) = COV.PRIOR.EPSILON*Eigen::Matrix3d::Identity();
    P_.block<3, 3>(INDEX_ERROR_ACCEL, INDEX_ERROR_ACCEL) = COV.PRIOR.DELTA*Eigen::Matrix3d::Identity();
}

/**
 * @brief  update observability analysis for pose measurement
 * @param  void
 * @return void
 */
void ErrorStateKalmanFilter::UpdateObservabilityAnalysisPose(
    const double &time, std::vector<double> &record
) {
    // build observability matrix for position measurement:
    for (int i = 1; i < DIM_STATE; ++i) {
        SOMPose_.block<DIM_MEASUREMENT_POSE, DIM_STATE>(i*DIM_MEASUREMENT_POSE, 0) = (
            SOMPose_.block<DIM_MEASUREMENT_POSE, DIM_STATE>((i - 1)*DIM_MEASUREMENT_POSE, 0) * F_
        );
    }

    // perform SVD analysis:
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(SOMPose_, Eigen::ComputeFullV);

    // record timestamp:
    record.push_back(time);

    // record singular values:
    for (int i = 0; i < DIM_STATE; ++i) {
        record.push_back(svd.singularValues()(i, 0));
    }

    // record degree of observability:
    // a. here assumes that in the latent space, there is a Gaussian white noise on each dim
    VectorX X = (
        svd.matrixV().cwiseAbs()*
        svd.singularValues().asDiagonal().inverse()
    ) * VectorX::Ones();
    // b. normalize:
    X = 100.0 * VectorX::Ones() - 100.0  / X.maxCoeff() * X;
    for (int i = 0; i < DIM_STATE; ++i) {
        record.push_back(X(i, 0));
    }
}

/**
 * @brief  update observability analysis for position measurement
 * @param  void
 * @return void
 */
void ErrorStateKalmanFilter::UpdateObservabilityAnalysisPosi(
    const double &time, std::vector<double> &record
) {
    // build observability matrix for position measurement:
    for (int i = 1; i < DIM_STATE; ++i) {
        SOMPosi_.block<DIM_MEASUREMENT_POSI, DIM_STATE>(i*DIM_MEASUREMENT_POSI, 0) = (
            SOMPosi_.block<DIM_MEASUREMENT_POSI, DIM_STATE>((i - 1)*DIM_MEASUREMENT_POSI, 0) * F_
        );
    }

    // perform SVD analysis:
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(SOMPosi_, Eigen::ComputeFullV);

    // record timestamp:
    record.push_back(time);

    // record singular values:
    for (int i = 0; i < DIM_STATE; ++i) {
        record.push_back(svd.singularValues()(i, 0));
    }

    // record degree of observability:
    // a. here assumes that in the latent space, there is a Gaussian white noise on each dim
    VectorX X = (
        svd.matrixV().cwiseAbs()*
        svd.singularValues().asDiagonal().inverse()
    ) * VectorX::Ones();
    // b. normalize:
    X = 100.0 * VectorX::Ones() - 100.0  / X.maxCoeff() * X;
    for (int i = 0; i < DIM_STATE; ++i) {
        record.push_back(X(i, 0));
    }
}

/**
 * @brief  update observability analysis for navigation position & body velocity measurement
 * @param  void
 * @return void
 */
void ErrorStateKalmanFilter::UpdateObservabilityAnalysisPosiVel(
    const double &time, std::vector<double> &record
) {
    // build observability matrix for position measurement:
    SOMPosiVel_.block<DIM_MEASUREMENT_POSI_VEL, DIM_STATE>(0, 0) = GPosiVel_;
    for (int i = 1; i < DIM_STATE; ++i) {
        SOMPosiVel_.block<DIM_MEASUREMENT_POSI_VEL, DIM_STATE>(i*DIM_MEASUREMENT_POSI_VEL, 0) = (
            SOMPosiVel_.block<DIM_MEASUREMENT_POSI_VEL, DIM_STATE>((i - 1)*DIM_MEASUREMENT_POSI_VEL, 0) * F_
        );
    }

    // perform SVD analysis:
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(SOMPosiVel_, Eigen::ComputeFullV);

    // record timestamp:
    record.push_back(time);

    // record singular values:
    for (int i = 0; i < DIM_STATE; ++i) {
        record.push_back(svd.singularValues()(i, 0));
    }

    // record degree of observability:
    // a. here assumes that in the latent space, there is a Gaussian white noise on each dim
    VectorX X = (
        svd.matrixV().cwiseAbs()*
        svd.singularValues().asDiagonal().inverse()
    ) * VectorX::Ones();
    // b. normalize:
    X = 100.0 * VectorX::Ones() - 100.0  / X.maxCoeff() * X;
    for (int i = 0; i < DIM_STATE; ++i) {
        record.push_back(X(i, 0));
    }
}

/**
 * @brief  update observability analysis
 * @param  measurement_type, measurement type
 * @return void
 */
void ErrorStateKalmanFilter::UpdateObservabilityAnalysis(
    const double &time,
    const MeasurementType &measurement_type
) {
    // init record:
    std::vector<double> record;

    switch ( measurement_type ) {
        case MeasurementType::POSE:
            UpdateObservabilityAnalysisPose(time, record);
            observability.pose_.push_back(record);
            break;
        case MeasurementType::POSI:
            UpdateObservabilityAnalysisPosi(time, record);
            observability.posi_.push_back(record);
            break;
        case MeasurementType::POSI_VEL:
            UpdateObservabilityAnalysisPosiVel(time, record);
            observability.posi_vel_.push_back(record);
            break;
        default:
            break;
    }
}

void ErrorStateKalmanFilter::SaveObservabilityAnalysis(
    const MeasurementType &measurement_type
) {
    std::vector<std::vector<double>> *data = nullptr;
    std::string type;

    switch ( measurement_type ) {
        case MeasurementType::POSE:
            data = &(observability.pose_);
            type = std::string("pose");
            break;
        case MeasurementType::POSI:
            data = &(observability.posi_);
            type = std::string("position");
            break;
        case MeasurementType::POSI_VEL:
            data = &(observability.posi_vel_);
            type = std::string("position_velocity");
            break;
        default:
            data = &(observability.posi_vel_);
            type = std::string("position_velocity");
            break;
    }

    // init:
    CSVWriter csv(",");
    csv.enableAutoNewRow(1 + 2*DIM_STATE);

    // a. write header:
    csv << "T";
    for (int i = 0; i < DIM_STATE; ++i) {
        csv << ("sv" + std::to_string(i + 1)); 
    }
    for (int i = 0; i < DIM_STATE; ++i) {
        csv << ("doo" + std::to_string(i + 1)); 
    }

    // b. write contents:
    for (const auto &record: *data) {
        // cast timestamp to int:
        csv << static_cast<int>(record.at(0));

        for (size_t i = 1; i < record.size(); ++i) {
            csv << std::fabs(record.at(i));
        }    
    }

    // save to persistent storage:
    csv.writeToFile(
        WORK_SPACE_PATH + "/slam_data/observability/" + type + ".csv"
    );
}

} // namespace lidar_localization