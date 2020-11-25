/*
 * @Description: Extended Kalman Filter for IMU-Lidar-GNSS-Odo-Mag fusion
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */

#ifndef LIDAR_LOCALIZATION_MODELS_KALMAN_FILTER_EXTENDED_KALMAN_FILTER_HPP_
#define LIDAR_LOCALIZATION_MODELS_KALMAN_FILTER_EXTENDED_KALMAN_FILTER_HPP_

#include <yaml-cpp/yaml.h>

#include <deque>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "lidar_localization/sensor_data/imu_data.hpp"

namespace lidar_localization {

class ExtendedKalmanFilter {
public:
    enum MeasurementType {
        POSE = 0,
        POSITION,
        POSITION_VELOCITY,
        POSITION_MAG,
        POSITION_VELOCITY_MAG,
        NUM_TYPES
    };

    struct Measurement {
        double time;
        
        // a. pose observation, lidar/visual frontend:
        Eigen::Matrix4d T_nb;
        // b. body frame velocity observation, odometer:
        Eigen::Vector3d v_b;
        // c. magnetometer:
        Eigen::Vector3d m_b;
    };

    struct Cov {
        struct {
            double x;
            double y;
            double z;
        } pos;
        struct {
            double x;
            double y;
            double z;
        } vel;
        // here quaternion is used for orientation representation:
        struct {
            double w;
            double x;
            double y;
            double z;
        } ori;
        struct {
            double x;
            double y;
            double z;
        } gyro_bias;
        struct {
            double x;
            double y;
            double z;
        } accel_bias;
    };

    // dimensions:
    static const int DIM_STATE = 16;
    static const int DIM_PROCESS_NOISE = 6;

    static const int DIM_MEASUREMENT_POSI = 3;
    static const int DIM_MEASUREMENT_POSI_NOISE = 3;
    static const int DIM_MEASUREMENT_POSI_MAG = 6;
    static const int DIM_MEASUREMENT_POSI_MAG_NOISE = 6;

    // indices:
    static const int INDEX_POS = 0;
    static const int INDEX_VEL = 3;
    static const int INDEX_ORI = 6;
    static const int INDEX_GYRO_BIAS = 10;
    static const int INDEX_ACCEL_BIAS = 13;
    
    // state:
    typedef Eigen::Matrix<double,                      DIM_STATE,                              1> VectorX;
    typedef Eigen::Matrix<double,                      DIM_STATE,                      DIM_STATE> MatrixP;
    // process equation:
    typedef Eigen::Matrix<double,                      DIM_STATE,                      DIM_STATE> MatrixF;
    typedef Eigen::Matrix<double,                      DIM_STATE,              DIM_PROCESS_NOISE> MatrixB;
    typedef Eigen::Matrix<double,              DIM_PROCESS_NOISE,              DIM_PROCESS_NOISE> MatrixQ;
    // measurement equation:
    typedef Eigen::Matrix<double,           DIM_MEASUREMENT_POSI,                      DIM_STATE> MatrixGPosi;
    typedef Eigen::Matrix<double,       DIM_MEASUREMENT_POSI_MAG,                      DIM_STATE> MatrixGPosiMag;

    typedef Eigen::Matrix<double,           DIM_MEASUREMENT_POSI,     DIM_MEASUREMENT_POSI_NOISE> MatrixCPosi;
    typedef Eigen::Matrix<double,       DIM_MEASUREMENT_POSI_MAG, DIM_MEASUREMENT_POSI_MAG_NOISE> MatrixCPosiMag;

    typedef Eigen::Matrix<double,     DIM_MEASUREMENT_POSI_NOISE,     DIM_MEASUREMENT_POSI_NOISE> MatrixRPosi;
    typedef Eigen::Matrix<double, DIM_MEASUREMENT_POSI_MAG_NOISE, DIM_MEASUREMENT_POSI_MAG_NOISE> MatrixRPosiMag;

    // measurement:
    typedef Eigen::Matrix<double,           DIM_MEASUREMENT_POSI,                              1> VectorYPosi;
    typedef Eigen::Matrix<double,       DIM_MEASUREMENT_POSI_MAG,                              1> VectorYPosiMag;
    // Kalman gain:
    typedef Eigen::Matrix<double,                      DIM_STATE,           DIM_MEASUREMENT_POSI> MatrixKPosi;
    typedef Eigen::Matrix<double,                      DIM_STATE,       DIM_MEASUREMENT_POSI_MAG> MatrixKPosiMag;

    // state observality matrix:
    typedef Eigen::Matrix<double, DIM_STATE*    DIM_MEASUREMENT_POSI, DIM_STATE> MatrixSOMPosi;
    typedef Eigen::Matrix<double, DIM_STATE*DIM_MEASUREMENT_POSI_MAG, DIM_STATE> MatrixSOMPosi;

    ExtendedKalmanFilter(const YAML::Node& node);

    /**
     * @brief  init filter
     * @param  imu_data, input IMU measurements
     * @return true if success false otherwise
     */
    void Init(
        const Eigen::Vector3d &vel,
        const IMUData &imu_data
    );

    /**
     * @brief  Kalman update
     * @param  imu_data, input IMU measurements
     * @return true if success false otherwise
     */
    bool Update(
        const IMUData &imu_data
    );

    /**
     * @brief  Kalman correction, pose measurement
     * @param  measurement_type, input measurement type
     * @param  measurement, input measurement
     * @return void                                   
     */
    bool Correct(
        const IMUData &imu_data, 
        const MeasurementType &measurement_type, const Measurement &measurement
    );

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

    /**
     * @brief  get covariance estimation
     * @param  cov, covariance output
     * @return void
     */
    void GetCovariance(Cov &cov);
    
    /**
     * @brief  update observability analysis
     * @param  time, measurement time
     * @param  measurement_type, measurement type
     * @return void
     */
    void UpdateObservabilityAnalysis(
        const double &time,
        const MeasurementType &measurement_type
    );

    /**
     * @brief  save observability analysis to persistent storage
     * @param  measurement_type, measurement type
     * @return void
     */
    void SaveObservabilityAnalysis(
        const MeasurementType &measurement_type
    );

private:
    /**
     * @brief  remove gravity component from accel measurement
     * @param  f_b, accel measurement measurement
     * @param  C_nb, orientation matrix
     * @return f_b
     */
    Eigen::Vector3d RemoveGravity(
        const Eigen::Vector3d &f_b,
        const Eigen::Matrix3d &C_nb
    );
    /**
     * @brief  get block matrix for velocity update by orientation quaternion
     * @param  f_b, accel measurement
     * @param  q_nb, orientation quaternion
     * @return block matrix Fvq
     */
    Eigen::Matrix<double, 3, 4> GetFVelOri(
        const Eigen::Vector3d &f_b,
        const Eigen::Quaterniond &q_nb
    );
    /**
     * @brief  get block matrix for orientation quaternion update by orientation quaternion
     * @param  w_b, gyro measurement
     * @return block matrix Fqq
     */
    Eigen::Matrix<double, 4, 4> GetFOriOri(
        const Eigen::Vector3d &w_b
    );
    /**
     * @brief  get block matrix for orientation quaternion update by epsilon, angular velocity bias
     * @param  q_nb, orientation quaternion
     * @return block matrix Fqe
     */
    Eigen::Matrix<double, 4, 3> GetFOriEps(
        const Eigen::Quaterniond &q_nb
    );
    /**
     * @brief  set process equation
     * @param  void
     * @return void
     */
    void SetProcessEquation(void);
    /**
     * @brief  update state estimation
     * @param  void
     * @return void
     */
    void UpdateStateEstimation(void);

    /**
     * @brief  correct state estimation using GNSS position
     * @param  T_nb, input GNSS position
     * @return void
     */
    void CorrectErrorEstimationPosi(const Eigen::Matrix4d &T_nb);

    /**
     * @brief  correct state estimation using GNSS position and magneto measurement
     * @param  T_nb, input GNSS position 
     * @param  B_b, input magneto
     * @return void
     */
    void CorrectStateEstimationPosiMag(const Eigen::Matrix4d &T_nb, const Eigen::Vector3d &B_b);

    /**
     * @brief  correct state estimation
     * @param  measurement_type, measurement type
     * @param  measurement, input measurement
     * @return void
     */
    void CorrectErrorEstimation(
        const MeasurementType &measurement_type, 
        const Measurement &measurement
    );

    /**
     * @brief  eliminate error
     * @param  void
     * @return void
     */
    void EliminateError(void);

    /**
     * @brief  is covariance stable
     * @param  INDEX_OFSET, state index offset
     * @param  THRESH, covariance threshold, defaults to 1.0e-5
     * @return void
     */
    bool IsCovStable(const int INDEX_OFSET, const double THRESH = 1.0e-5);

    /**
     * @brief  reset filter state
     * @param  void
     * @return void
     */
    void ResetState(void);
    /**
     * @brief  reset filter covariance
     * @param  void
     * @return void
     */
    void ResetCovariance(void);
    /**
     * @brief  update observability analysis for pose measurement
     * @param  void
     * @return void
     */
    void UpdateObservabilityAnalysisPose(
        const double &time, std::vector<double> &record
    );

    /**
     * @brief  update observability analysis for position measurement
     * @param  void
     * @return void
     */
    void UpdateObservabilityAnalysisPosition(
        const double &time, std::vector<double> &record
    );

    /**
     * @brief  update observability analysis for navigation position & body velocity measurement
     * @param  void
     * @return void
     */
    void UpdateObservabilityAnalysisPosVel(
        const double &time, std::vector<double> &record
    );

    // data buff:
    std::deque<IMUData> imu_data_buff_;

    // time:
    double time_;

    // init pose, vel, gyro & accel bias:
    Eigen::Matrix4d init_pose_ = Eigen::Matrix4d::Identity();
    Eigen::Vector3d init_vel_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_bias_ = Eigen::Vector3d::Zero();

    // state:
    VectorX X_ = VectorX::Zero();
    MatrixP P_ = MatrixP::Zero();
    // process & measurement equations:
    MatrixF F_ = MatrixF::Zero();
    MatrixB B_ = MatrixB::Zero();
    MatrixQ Q_ = MatrixQ::Zero();

    MatrixGPosition GPosi_ = MatrixGPosi::Zero();
    MatrixGPosiMag GPosiMag_ = MatrixGPosiMag::Zero();

    MatrixCPosition CPosi_ = MatrixCPosi::Zero();
    MatrixCPosiMag CPosiMag_ = MatrixCPosiMag::Zero();

    MatrixRPosition RPosi_ = MatrixRPosi::Zero();
    MatrixRPosiMag RPosiMag_ = MatrixRPosiMag::Zero();

    MatrixSOMPosi SOMPosi_ = MatrixSOMPosi::Zero();
    MatrixSOMPosiMag SOMPosiMag_ = MatrixSOMPosiMag::Zero();

    // measurement:
    VectorYPosi YPosi_;
    VectorYPosiMag YPosiMag_;

    // earth constants:
    Eigen::Vector3d g_;
    Eigen::Vector3d w_;

    // observability analysis:
    struct {
        std::vector<std::vector<double>> pose_;
        std::vector<std::vector<double>> posi_;
        std::vector<std::vector<double>> posi_mag_;
    } observability;

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
            double ORI;
            double EPSILON;
            double DELTA;
        } PRIOR;
        struct {
            double GYRO;
            double ACCEL;
        } PROCESS;
        struct {
            double POSI;
            double MAG;
        } MEASUREMENT;
    } COV;
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_MODELS_KALMAN_FILTER_EXTENDED_KALMAN_FILTER_HPP_