/*
 * @Description: Extended Kalman Filter for IMU-Lidar-GNSS-Odo-Mag fusion
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

#include "lidar_localization/models/kalman_filter/extended_kalman_filter.hpp"

#include "lidar_localization/global_defination/global_defination.h"

#include "lidar_localization/tools/CSVWriter.hpp"

#include "glog/logging.h"

namespace lidar_localization {

ExtendedKalmanFilter::ExtendedKalmanFilter(const YAML::Node& node) {
    //
    // parse config:
    // 
    // a. earth constants:
    EARTH.GRAVITY_MAGNITUDE = node["earth"]["gravity_magnitude"].as<double>();
    EARTH.ROTATION_SPEED = node["earth"]["rotation_speed"].as<double>();
    EARTH.LATITUDE = node["earth"]["latitude"].as<double>();
    EARTH.LATITUDE *= M_PI / 180.0;
    // b. prior state covariance:
    COV.PRIOR.POS = node["covariance"]["prior"]["pos"].as<double>();
    COV.PRIOR.VEL = node["covariance"]["prior"]["vel"].as<double>();
    // TODO: find a better way for quaternion orientation prior covariance assignment
    COV.PRIOR.ORI = node["covariance"]["prior"]["ori"].as<double>();
    COV.PRIOR.EPSILON = node["covariance"]["prior"]["epsilon"].as<double>();
    COV.PRIOR.DELTA = node["covariance"]["prior"]["delta"].as<double>();
    // c. process noise:
    COV.PROCESS.GYRO = node["covariance"]["process"]["gyro"].as<double>();
    COV.PROCESS.ACCEL = node["covariance"]["process"]["accel"].as<double>();
    // d. measurement noise:
    COV.MEASUREMENT.POSI = node["covariance"]["measurement"]["posi"].as<double>();
    COV.MEASUREMENT.MAG = node["covariance"]["measurement"]["mag"].as<double>();

    // prompt:
    LOG(INFO) << std::endl 
              << "Iterative Extended Kalman Filter params:" << std::endl
              << "\tgravity magnitude: " << EARTH.GRAVITY_MAGNITUDE << std::endl
              << "\tearth rotation speed: " << EARTH.ROTATION_SPEED << std::endl
              << "\tlatitude: " << EARTH.LATITUDE << std::endl
              << std::endl
              << "\tprior cov. pos.: " << COV.PRIOR.POS  << std::endl
              << "\tprior cov. vel.: " << COV.PRIOR.VEL << std::endl
              << "\tprior cov. ori: " << COV.PRIOR.ORI << std::endl
              << "\tprior cov. epsilon.: " << COV.PRIOR.EPSILON  << std::endl
              << "\tprior cov. delta.: " << COV.PRIOR.DELTA << std::endl
              << std::endl
              << "\tprocess noise gyro.: " << COV.PROCESS.GYRO << std::endl
              << "\tprocess noise accel.: " << COV.PROCESS.ACCEL << std::endl
              << std::endl
              << "\tmeasurement noise posi.: " << COV.MEASUREMENT.POSI << std::endl
              << "\tmeasurement noise mag.: " << COV.MEASUREMENT.MAG << std::endl
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
    RPosi_.block<3, 3>(0, 0) = COV.MEASUREMENT.POSI*Eigen::Matrix3d::Identity();

    RPosiMag_.block<3, 3>(0, 0) = COV.MEASUREMENT.POSI*Eigen::Matrix3d::Identity();
    RPosiMag_.block<3, 3>(3, 3) = COV.MEASUREMENT.MAG*Eigen::Matrix3d::Identity();

    // e. process equation:
    F_.block<3, 3>( INDEX_POS, INDEX_VEL ) = Eigen::Matrix3d::Identity();

    // f. measurement equation:
    GPosi_.block<3, 3>( 0, INDEX_POS ) = Eigen::Matrix3d::Identity();

    GPosiMag_.block<3, 3>( 0, INDEX_POS ) = Eigen::Matrix3d::Identity();

    // init soms:
    SOMPosi_.block<DIM_MEASUREMENT_POSI, DIM_STATE>(0, 0) = GPosi_;
    SOMPosiMag_.block<DIM_MEASUREMENT_POSI_MAG, DIM_STATE>(0, 0) = GPosiMag_;
}

/**
 * @brief  init filter
 * @param  pose, init pose
 * @param  vel, init vel
 * @param  imu_data, init IMU measurements
 * @return true if success false otherwise
 */
void ExtendedKalmanFilter::Init(
    const Eigen::Vector3d &vel,
    const IMUData &imu_data
) {
    // get init C_nb from IMU estimation:
    Eigen::Matrix3d C_nb = imu_data.GetOrientationMatrix().cast<double>();
    Eigen::Vector4d q_nb(
        imu_data.orientation.w, 
        imu_data.orientation.x, 
        imu_data.orientation.y, 
        imu_data.orientation.z
    );
    // get init v_n from v_b:
    Eigen::Vector3d v_n = C_nb*vel;

    // set init pose:
    init_pose_.block<3, 3>(0, 0) = C_nb;

    // set init velocity:
    init_vel_ = v_n;

    // set init state:
    X_.block<3, 1>(INDEX_POS, 0) = init_pose_.block<3, 1>(0, 3);
    X_.block<3, 1>(INDEX_VEL, 0) = v_n;
    X_.block<4, 1>(INDEX_ORI, 0) = q_nb;
    X_.block<3, 1>(INDEX_GYRO_BIAS, 0) = gyro_bias_;
    X_.block<3, 1>(INDEX_ACCEL_BIAS, 0) = accel_bias_;

    // init IMU data buffer:
    imu_data_buff_.clear();
    imu_data_buff_.push_back(imu_data);

    // init filter time:
    time_ = imu_data.time;

    // init process equation, in case of direct correct step:
    UpdateProcessEquation(linear_acc_init);

    LOG(INFO) << std::endl 
              << "Iterative Extended Kalman Filter Inited at " << static_cast<int>(time_) << std::endl
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
bool ExtendedKalmanFilter::Update(const IMUData &imu_data) {
    // update IMU buff:
    if (time_ < imu_data.time) {
        // update buffer:
        imu_data_buff_.pop_front();
        imu_data_buff_.push_back(imu_data);

        // update prediction:
        UpdateStateEstimation();
        
        // move forward:
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
bool ExtendedKalmanFilter::Correct(
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
void ExtendedKalmanFilter::GetOdometry(
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
void ExtendedKalmanFilter::GetCovariance(Cov &cov) {
    static int OFFSET_X = 0;
    static int OFFSET_Y = 1;
    static int OFFSET_Z = 2;

    // a. delta position:
    cov.delta_pos.x = P_(INDEX_ERROR_POS + OFFSET_X, INDEX_ERROR_POS + OFFSET_X);
    cov.delta_pos.y = P_(INDEX_ERROR_POS + OFFSET_Y, INDEX_ERROR_POS + OFFSET_Y);
    cov.delta_pos.z = P_(INDEX_ERROR_POS + OFFSET_Z, INDEX_ERROR_POS + OFFSET_Z);

    // b. delta velocity:
    cov.delta_vel.x = P_(INDEX_ERROR_VEL + OFFSET_X, INDEX_ERROR_VEL + OFFSET_X);
    cov.delta_vel.y = P_(INDEX_ERROR_VEL + OFFSET_Y, INDEX_ERROR_VEL + OFFSET_Y);
    cov.delta_vel.z = P_(INDEX_ERROR_VEL + OFFSET_Z, INDEX_ERROR_VEL + OFFSET_Z);

    // c. delta orientation:
    cov.delta_ori.x = P_(INDEX_ERROR_ORI + OFFSET_X, INDEX_ERROR_ORI + OFFSET_X);
    cov.delta_ori.y = P_(INDEX_ERROR_ORI + OFFSET_Y, INDEX_ERROR_ORI + OFFSET_Y);
    cov.delta_ori.z = P_(INDEX_ERROR_ORI + OFFSET_Z, INDEX_ERROR_ORI + OFFSET_Z);

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
 * @brief  remove gravity component from accel measurement
 * @param  f_b, accel measurement measurement
 * @param  C_nb, orientation matrix
 * @return f_b
 */
Eigen::Vector3d ExtendedKalmanFilter::RemoveGravity(
    const Eigen::Vector3d &f_b,
    const Eigen::Matrix3d &C_nb
) {
    return f_b - C_nb.transpose()*g_;
}

/**
 * @brief  get block matrix for velocity update by orientation quaternion
 * @param  f_b, accel measurement
 * @param  q_nb, orientation quaternion
 * @return block matrix Fvq
 */
Eigen::Matrix<double, 3, 4> ExtendedKalmanFilter::GetFVelOri(
    const Eigen::Vector3d &f_b,
    const Eigen::Quaterniond &q_nb
) {
    // get F:
    Eigen::Matrix<double, 4, 3> T_Ff;
    T_Ff << +q_nb.w(), -q_nb.z(), +q_nb.y(),
            +q_nb.x(), +q_nb.y(), +q_nb.z(),
            -q_nb.y(), +q_nb.x(), +q_nb.w(),
            -q_nb.z(), -q_nb.w(), +q_nb.x();
    Eigen::Vector4d F = 2 * T_Ff * f_b;

    // get Fvq:
    Eigen::Matrix<double, 3, 4> Fvq;
    Fvq << +F(0), +F(1), +F(2), +F(3),
           -F(3), -F(2), +F(1), +F(0),
           +F(2), -F(3), -F(0), +F(1);

    return Fvq;
}

/**
 * @brief  get block matrix for orientation quaternion update by orientation quaternion
 * @param  w_b, gyro measurement
 * @return block matrix Fqq
 */
Eigen::Matrix<double, 4, 4> ExtendedKalmanFilter::GetFOriOri(
    const Eigen::Vector3d &w_b
) {
    // get Fqq:
    Eigen::Matrix<double, 4, 4> Fqq;
    Fqq <<      0.0, -w_b.x(), -w_b.y(), -w_b.z(),
           +w_b.x(),      0.0, +w_b.z(), -w_b.y(),
           +w_b.y(), -w_b.z(),      0.0, +w_b.x(),
           +w_b.z(), +w_b.y(), -w_b.x(),      0.0;

    return 0.5 * Fqq;
}

/**
 * @brief  get block matrix for orientation quaternion update by epsilon, angular velocity bias
 * @param  q_nb, orientation quaternion
 * @return block matrix Fqe
 */
Eigen::Matrix<double, 4, 3> ExtendedKalmanFilter::GetFOriEps(
    const Eigen::Quaterniond &q_nb
) {
    // get Fqe:
    Eigen::Matrix<double, 4, 3> Fqe;
    Fqe << -q_nb.x(), -q_nb.y(), -q_nb.z(),
           +q_nb.w(), -q_nb.z(), +q_nb.y(),
           +q_nb.z(), +q_nb.w(), -q_nb.x(),
           -q_nb.y(), +q_nb.x(), +q_nb.w();

    return 0.5 * Fqe;
}

/**
 * @brief  set process equation
 * @param  void
 * @return void
 */
void ExtendedKalmanFilter::SetProcessEquation(void) {
    // parse IMU measurement:
    const IMUData &imu_data = imu_data_buff_.front();

    const Eigen::Vector3d f_b(
        imu_data.linear_acceleration.x,
        imu_data.linear_acceleration.y,
        imu_data.linear_acceleration.z
    );
    const Eigen::Vector3d w_b(
        imu_data.angular_velocity.x,
        imu_data.angular_velocity.y,
        imu_data.angular_velocity.z
    );

    // parse orientation:
    const Eigen::Quaterniond q_nb(
        X_(INDEX_ORI + 0, 0),
        X_(INDEX_ORI + 1, 0),
        X_(INDEX_ORI + 2, 0),
        X_(INDEX_ORI + 3, 0)
    );
    const Eigen::Matrix3d C_nb = q_nb.toRotationMatrix();

    //
    // EKF is linearized around VectorX::Zero
    //
    // a. set equation for velocity:
    f_b = RemoveGravity(f_b, C_nb);
    F_.block<3, 4>(INDEX_VEL,         INDEX_ORI) = GetFVelOri(f_b, q_nb);
    F_.block<3, 3>(INDEX_VEL,  INDEX_ACCEL_BIAS) = B_.block<3, 3>(INDEX_VEL, 3) = C_nb;
    
    // b. set equation for orientation quaternion:
    F_.block<4, 4>(INDEX_ORI,         INDEX_ORI) = GetFOriOri(w_b);
    F_.block<4, 3>(INDEX_ORI, INDEX_ERROR_ACCEL) = B_.block<4, 3>(INDEX_ORI, 0) = GetFOriEps(q_nb);
}

/**
 * @brief  update state estimation
 * @param  void
 * @return void
 */
void ExtendedKalmanFilter::UpdateStateEstimation(void) {
    static int count = 0;

    static MatrixF F_1st;
    static MatrixF F_2nd;

    // update process equation:
    SetProcessEquation();

    // get discretized process equations:
    double T = imu_data_buff_.front().time - time_;

    // approximate to 2nd order:
    F_1st = T*F_;
    F_2nd = 0.5*T*F_*F_1st;
    MatrixF F = MatrixF::Identity() + F_1st + F_2nd;

    MatrixB B = T*B_;

    // perform Kalman prediction:
    X_ = F*X_;
    P_ = F*P_*F.transpose() + B*Q_*B.transpose();

    if (
        0 == (++count % 10) 
    ) {
        count = 0;
        /*
        // covariance monitor, update step, debugging only:
        LOG(INFO) << std::endl 
                  << static_cast<int>(imu_data.time) << " Update: " << std::endl
                  << P_( 0,  0) << ", " << P_( 1,  1) << ", " << P_( 2,  2) << std::endl
                  << P_( 3,  3) << ", " << P_( 4,  4) << ", " << P_( 5,  5) << std::endl
                  << P_( 6,  6) << ", " << P_( 7,  7) << ", " << P_( 8,  8) << P_( 9,  9) << std::endl
                  << P_(10, 10) << ", " << P_(11, 11) << ", " << P_(12, 12) << std::endl
                  << P_(13, 13) << ", " << P_(14, 14) << ", " << P_(15, 15) << std::endl
                  << std::endl;
        */
    }
}

/**
 * @brief  correct error estimation using position measurement
 * @param  T_nb, input position measurement
 * @return void
 */
void ExtendedKalmanFilter::CorrectErrorEstimationPosition(
    const Eigen::Matrix4d &T_nb
) {
    // parse measurement:
    Eigen::Vector3d P_nn_obs = pose_.block<3, 1>(0,3) - T_nb.block<3, 1>(0,3);

    YPosition_.block<3, 1>(0, 0) = P_nn_obs;

    // build Kalman gain:
    MatrixRPosition R = GPosition_*P_*GPosition_.transpose() + RPosition_;
    MatrixKPosition K = P_*GPosition_.transpose()*R.inverse();

    // perform Kalman correct:
    P_ = (MatrixP::Identity() - K*GPosition_)*P_;
    X_ = X_ + K*(YPosition_ - GPosition_*X_);
}

/**
 * @brief  correct error estimation using navigation position and body velocity measurement
 * @param  T_nb, input position measurement
 * @param  v_b, input velocity measurement
 * @return void
 */
void ExtendedKalmanFilter::CorrectErrorEstimationPosVel(
    const Eigen::Matrix4d &T_nb, const Eigen::Vector3d &v_b
) {
    // parse measurement:
    Eigen::Vector3d P_nn_obs = pose_.block<3, 1>(0,3) - T_nb.block<3, 1>(0,3);
    Eigen::Vector3d v_bb_obs = pose_.block<3, 3>(0,0).transpose()*vel_ - v_b;
    // apply motion constraint:
    v_bb_obs.y() = v_bb_obs.z() = 0.0;
    // Eigen::Matrix3d C_nn_obs = pose_.block<3, 3>(0,0) * T_nb.block<3, 3>(0,0).transpose();

    YPosVel_.block<3, 1>(0, 0) = P_nn_obs;
    YPosVel_.block<3, 1>(3, 0) = v_bb_obs ;
    // YPosVel_.block<3, 1>(6, 0) = Sophus::SO3d::vee(Eigen::Matrix3d::Identity() - C_nn_obs);

    // set measurement equation:
    GPosVel_.block<3, 3>(3, INDEX_ERROR_VEL) =  pose_.block<3, 3>(0,0).transpose();
    GPosVel_.block<3, 3>(3, INDEX_ERROR_ORI) = -pose_.block<3, 3>(0,0).transpose()*Sophus::SO3d::hat(vel_);

    // build Kalman gain:
    MatrixRPosVel R = GPosVel_*P_*GPosVel_.transpose() + RPosVel_;
    MatrixKPosVel K = P_*GPosVel_.transpose()*R.inverse();

    // perform Kalman correct:
    P_ = (MatrixP::Identity() - K*GPosVel_)*P_;
    X_ = X_ + K*(YPosVel_ - GPosVel_*X_);
}

/**
 * @brief  correct state estimation
 * @param  measurement_type, measurement type
 * @param  measurement, input measurement
 * @return void
 */
void ExtendedKalmanFilter::CorrectErrorEstimation(
    const MeasurementType &measurement_type, 
    const Measurement &measurement
) {
    static int count = 0;

    switch ( measurement_type ) {
        case MeasurementType::POSE:
            CorrectErrorEstimationPose(measurement.T_nb);
            break;
        case MeasurementType::POSITION:
            CorrectErrorEstimationPosition(measurement.T_nb);
            break;
        case MeasurementType::POSITION_VELOCITY:
            CorrectErrorEstimationPosVel(
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
void ExtendedKalmanFilter::EliminateError(void) {
    // a. position:
    pose_.block<3, 1>(0, 3) = pose_.block<3, 1>(0, 3) - X_.block<3, 1>(INDEX_ERROR_POS, 0);
    // b. velocity:
    vel_ = vel_ - X_.block<3, 1>(INDEX_ERROR_VEL, 0);

    Eigen::Vector3d vel_b_ = pose_.block<3, 3>(0, 0).transpose() * vel_;
    vel_b_.y() = vel_b_.z() = 0.0;
    vel_ = pose_.block<3, 3>(0, 0) * vel_b_;
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
bool ExtendedKalmanFilter::IsCovStable(
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
void ExtendedKalmanFilter::ResetState(void) {
    // reset current state:
    X_ = VectorX::Zero();
}

/**
 * @brief  reset filter covariance
 * @param  void
 * @return void
 */
void ExtendedKalmanFilter::ResetCovariance(void) {
    P_ = MatrixP::Zero();
    
    P_.block<3, 3>(       INDEX_POS,        INDEX_POS) = COV.PRIOR.POS*Eigen::Matrix3d::Identity();
    P_.block<3, 3>(       INDEX_VEL,        INDEX_VEL) = COV.PRIOR.VEL*Eigen::Matrix3d::Identity();
    // TODO: find a better way for quaternion orientation prior covariance assignment
    P_.block<4, 4>(       INDEX_ORI,        INDEX_ORI) = COV.PRIOR.ORI*Eigen::Matrix3d::Identity();
    P_.block<3, 3>( INDEX_GYRO_BIAS,  INDEX_GYRO_BIAS) = COV.PRIOR.EPSILON*Eigen::Matrix3d::Identity();
    P_.block<3, 3>(INDEX_ACCEL_BIAS, INDEX_ACCEL_BIAS) = COV.PRIOR.DELTA*Eigen::Matrix3d::Identity();
}

/**
 * @brief  update observability analysis for pose measurement
 * @param  void
 * @return void
 */
void ExtendedKalmanFilter::UpdateObservabilityAnalysisPose(
    const double &time, std::vector<double> &record
) {
    ;
}

/**
 * @brief  update observability analysis for position measurement
 * @param  void
 * @return void
 */
void ExtendedKalmanFilter::UpdateObservabilityAnalysisPosition(
    const double &time, std::vector<double> &record
) {
    // build observability matrix for position measurement:
    for (int i = 1; i < DIM_STATE; ++i) {
        SOMPosition_.block<DIM_MEASUREMENT_POSITION, DIM_STATE>(i*DIM_MEASUREMENT_POSITION, 0) = (
            SOMPosition_.block<DIM_MEASUREMENT_POSITION, DIM_STATE>((i - 1)*DIM_MEASUREMENT_POSITION, 0) * F_
        );
    }

    // perform SVD analysis:
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(SOMPosition_, Eigen::ComputeThinU | Eigen::ComputeThinV);

    // record timestamp:
    record.push_back(time);

    // record singular values:
    for (int i = 0; i < DIM_STATE; ++i) {
        record.push_back(svd.singularValues()(i, 0));
    }

    // record degree of observability:
    Eigen::Matrix<double, DIM_STATE*DIM_MEASUREMENT_POSITION, 1> Y = COV.MEASUREMENT.POS*Eigen::Matrix<double, DIM_STATE*DIM_MEASUREMENT_POSITION, 1>::Ones();
    VectorX X = (
        svd.matrixV()*
        svd.singularValues().asDiagonal().inverse()*
        svd.matrixU().transpose()
    )*Y;
    for (int i = 0; i < DIM_STATE; ++i) {
        record.push_back(X(i, 0));
    }
}

/**
 * @brief  update observability analysis for navigation position & body velocity measurement
 * @param  void
 * @return void
 */
void ExtendedKalmanFilter::UpdateObservabilityAnalysisPosVel(
    const double &time, std::vector<double> &record
) {
    // build observability matrix for position measurement:
    SOMPosVel_.block<DIM_MEASUREMENT_POSVEL, DIM_STATE>(0, 0) = GPosVel_;
    for (int i = 1; i < DIM_STATE; ++i) {
        SOMPosVel_.block<DIM_MEASUREMENT_POSVEL, DIM_STATE>(i*DIM_MEASUREMENT_POSVEL, 0) = (
            SOMPosVel_.block<DIM_MEASUREMENT_POSVEL, DIM_STATE>((i - 1)*DIM_MEASUREMENT_POSVEL, 0) * F_
        );
    }

    // perform SVD analysis:
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(SOMPosVel_, Eigen::ComputeThinU | Eigen::ComputeThinV);

    // record timestamp:
    record.push_back(time);

    // record singular values:
    for (int i = 0; i < DIM_STATE; ++i) {
        record.push_back(svd.singularValues()(i, 0));
    }

    // record degree of observability:
    Eigen::Matrix<double, DIM_STATE*DIM_MEASUREMENT_POSVEL, 1> Y = COV.MEASUREMENT.POS*Eigen::Matrix<double, DIM_STATE*DIM_MEASUREMENT_POSVEL, 1>::Ones();
    VectorX X = (
        svd.matrixV()*
        svd.singularValues().asDiagonal().inverse()*
        svd.matrixU().transpose()
    )*Y;
    for (int i = 0; i < DIM_STATE; ++i) {
        record.push_back(X(i, 0));
    }
}

/**
 * @brief  update observability analysis
 * @param  measurement_type, measurement type
 * @return void
 */
void ExtendedKalmanFilter::UpdateObservabilityAnalysis(
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
        case MeasurementType::POSITION:
            UpdateObservabilityAnalysisPosition(time, record);
            observability.position_.push_back(record);
            break;
        case MeasurementType::POSITION_VELOCITY:
            observability.pos_vel_.push_back(record);
            break;
        default:
            break;
    }
}

void ExtendedKalmanFilter::SaveObservabilityAnalysis(
    const MeasurementType &measurement_type
) {
    std::vector<std::vector<double>> *data = nullptr;
    std::string type;

    switch ( measurement_type ) {
        case MeasurementType::POSE:
            data = &(observability.pose_);
            type = std::string("pose");
            break;
        case MeasurementType::POSITION:
            data = &(observability.position_);
            type = std::string("position");
            break;
        case MeasurementType::POSITION_VELOCITY:
            data = &(observability.pos_vel_);
            type = std::string("position_velocity");
            break;
        default:
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