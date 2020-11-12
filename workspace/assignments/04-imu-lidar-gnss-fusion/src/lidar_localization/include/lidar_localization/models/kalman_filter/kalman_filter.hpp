/*
 * @Description: IMU-lidar-GNSS fusion using Kalman filter for localization
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */

#ifndef LIDAR_LOCALIZATION_MODELS_KALMAN_FILTER_HPP_
#define LIDAR_LOCALIZATION_MODELS_KALMAN_FILTER_HPP_

#include <yaml-cpp/yaml.h>

#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

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

    KalmanFilter(const YAML::Node& node);
private:
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