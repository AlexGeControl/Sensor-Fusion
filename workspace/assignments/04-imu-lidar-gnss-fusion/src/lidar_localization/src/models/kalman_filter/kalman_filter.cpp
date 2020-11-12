/*
 * @Description: IMU-lidar-GNSS fusion using Kalman filter for localization
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */
#include <limits>

#include <cmath>
#include <iostream>
#include <fstream>
#include <ostream>

#include "lidar_localization/models/kalman_filter/kalman_filter.hpp"

#include "glog/logging.h"

namespace lidar_localization {

KalmanFilter::KalmanFilter(const YAML::Node& node) {
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
    COV.PRIOR.ORIENTATION = node["covariance"]["prior"]["orientation"].as<double>();
    COV.PRIOR.EPSILON = node["covariance"]["prior"]["epsilon"].as<double>();
    COV.PRIOR.DELTA = node["covariance"]["prior"]["delta"].as<double>();
    // c. process noise:
    COV.PROCESS.GYRO = node["covariance"]["process"]["gyro"].as<double>();
    COV.PROCESS.ACCEL = node["covariance"]["process"]["accel"].as<double>();
    // d. measurement noise:
    COV.MEASUREMENT.POS = node["covariance"]["measurement"]["pos"].as<double>();
    COV.MEASUREMENT.ORIENTATION = node["covariance"]["measurement"]["orientation"].as<double>();

    // prompt:
    LOG(INFO) << std::endl 
              << "IMU-Lidar Kalman Filter params:" << std::endl
              << "\tgravity magnitude: " << EARTH.GRAVITY_MAGNITUDE << std::endl
              << "\tearth rotation speed: " << EARTH.ROTATION_SPEED << std::endl
              << "\tlatitude: " << EARTH.LATITUDE << std::endl
              << std::endl
              << "\tprior cov. pos.: " << COV.PRIOR.POS  << std::endl
              << "\tprior cov. vel.: " << COV.PRIOR.VEL << std::endl
              << "\tprior cov. ori: " << COV.PRIOR.ORIENTATION << std::endl
              << "\tprior cov. epsilon.: " << COV.PRIOR.EPSILON  << std::endl
              << "\tprior cov. delta.: " << COV.PRIOR.DELTA << std::endl
              << std::endl
              << "\tprocess noise gyro.: " << COV.PROCESS.GYRO << std::endl
              << "\tprocess noise accel.: " << COV.PROCESS.ACCEL << std::endl
              << std::endl
              << "\tmeasurement noise pos.: " << COV.MEASUREMENT.POS << std::endl
              << "\tmeasurement noise orientation.: " << COV.MEASUREMENT.ORIENTATION << std::endl
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
    // b. prior state covariance:
    P_.block<3, 3>(  INDEX_ERROR_POS,   INDEX_ERROR_POS) = COV.PRIOR.POS*Eigen::Matrix3d::Identity();
    P_.block<3, 3>(  INDEX_ERROR_VEL,   INDEX_ERROR_VEL) = COV.PRIOR.VEL*Eigen::Matrix3d::Identity();
    P_.block<3, 3>(  INDEX_ERROR_ORI,   INDEX_ERROR_ORI) = COV.PRIOR.ORIENTATION*Eigen::Matrix3d::Identity();
    P_.block<3, 3>( INDEX_ERROR_GYRO,  INDEX_ERROR_GYRO) = COV.PRIOR.EPSILON*Eigen::Matrix3d::Identity();
    P_.block<3, 3>(INDEX_ERROR_ACCEL, INDEX_ERROR_ACCEL) = COV.PRIOR.DELTA*Eigen::Matrix3d::Identity();
    // c. process noise:
    Q_.block<3, 3>(0, 0) = COV.PROCESS.GYRO*Eigen::Matrix3d::Identity();
    Q_.block<3, 3>(3, 3) = COV.PROCESS.ACCEL*Eigen::Matrix3d::Identity();
    // d. measurement noise:
    R_.block<3, 3>(0, 0) = COV.MEASUREMENT.POS*Eigen::Matrix3d::Identity();
    R_.block<3, 3>(3, 3) = COV.MEASUREMENT.ORIENTATION*Eigen::Matrix3d::Identity();
    // e. measurement equation:
    G_.block<3, 3>(0, INDEX_ERROR_POS) = Eigen::Matrix3d::Identity();
    G_.block<3, 3>(3, INDEX_ERROR_ORI) = Eigen::Matrix3d::Identity();
    C_.block<3, 3>(0, 0) = C_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
}

} // namespace lidar_localization