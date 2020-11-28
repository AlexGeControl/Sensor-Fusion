/*
 * @Description: Kalman Filter interface.
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */

#ifndef LIDAR_LOCALIZATION_MODELS_KALMAN_FILTER_KALMAN_FILTER_HPP_
#define LIDAR_LOCALIZATION_MODELS_KALMAN_FILTER_KALMAN_FILTER_HPP_

#include <yaml-cpp/yaml.h>

#include <deque>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "lidar_localization/sensor_data/imu_data.hpp"

namespace lidar_localization {

class KalmanFilter {
public:
    /**
     * @class MeasurementType
     * @brief enum for observation type
     */
    enum MeasurementType {
        POSE = 0,
        POSE_VEL,
        POSI,
        POSI_VEL,
        POSI_MAG,
        POSI_VEL_MAG,
        NUM_TYPES
    };

    /**
     * @class Measurement
     * @brief Kalman filter measurement data
     */
    struct Measurement {
        // timestamp:
        double time;
        // a. pose observation, lidar/visual frontend:
        Eigen::Matrix4d T_nb;
        // b. body frame velocity observation, odometer:
        Eigen::Vector3d v_b;
        // c. body frame angular velocity, needed by motion constraint:
        Eigen::Vector3d w_b;
        // d. magnetometer:
        Eigen::Vector3d B_b;
    };

    /**
     * @class Cov
     * @brief Kalman filter process covariance data
     */
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

    /**
     * @brief  init filter
     * @param  imu_data, input IMU measurements
     * @return true if success false otherwise
     */
    virtual void Init(
        const Eigen::Vector3d &vel, 
        const IMUData &imu_data
    ) = 0;

    /**
     * @brief  update state & covariance estimation, Kalman prediction
     * @param  imu_data, input IMU measurements
     * @return true if success false otherwise
     */
    virtual bool Update(
        const IMUData &imu_data
    ) = 0;

    /**
     * @brief  correct state & covariance estimation, Kalman correction
     * @param  measurement_type, input measurement type
     * @param  measurement, input measurement
     * @return void                                   
     */
    virtual bool Correct(
        const IMUData &imu_data, 
        const MeasurementType &measurement_type, const Measurement &measurement
    ) = 0;

    /**
     * @brief  get filter time
     * @return filter time as double    
     */
    double GetTime(void) const { return time_; }
    
    /**
     * @brief  get odometry estimation
     * @param  pose, output pose
     * @param  vel, output vel
     * @return void
     */
    virtual void GetOdometry(Eigen::Matrix4f &pose, Eigen::Vector3f &vel) = 0;

    /**
     * @brief  get covariance estimation
     * @param  cov, output covariance 
     * @return void
     */
    virtual void GetCovariance(Cov &cov) = 0;
    
    /**
     * @brief  update observability analysis
     * @param  time, measurement time
     * @param  measurement_type, measurement type
     * @return void
     */
    virtual void UpdateObservabilityAnalysis(
        const double &time,
        const MeasurementType &measurement_type
    ) = 0;

    /**
     * @brief  save observability analysis to persistent storage
     * @param  measurement_type, measurement type
     * @return void
     */
    virtual void SaveObservabilityAnalysis(
        const MeasurementType &measurement_type
    ) = 0;

protected:
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
    void SetProcessEquation(const IMUData &imu_data);
    /**
     * @brief  update covariance estimation
     * @param  void
     * @return void
     */
    void UpdateCovarianceEstimation(const IMUData &imu_data);

    /**
     * @brief  get unbiased angular velocity in body frame
     * @param  angular_vel, angular velocity measurement
     * @param  C_nb, corresponding orientation of measurement
     * @return unbiased angular velocity in body frame
     */
    inline Eigen::Vector3d GetUnbiasedAngularVel(
        const Eigen::Vector3d &angular_vel,
        const Eigen::Matrix3d &C_nb
    );
    /**
     * @brief  get unbiased linear acceleration in navigation frame
     * @param  linear_acc, linear acceleration measurement
     * @param  C_nb, corresponding orientation of measurement
     * @return unbiased linear acceleration in navigation frame
     */
    inline Eigen::Vector3d GetUnbiasedLinearAcc(
        const Eigen::Vector3d &linear_acc,
        const Eigen::Matrix3d &C_nb
    );
    /**
     * @brief  remove gravity component from accel measurement
     * @param  f_b, accel measurement measurement
     * @param  C_nb, orientation matrix
     * @return f_b
     */
    inline Eigen::Vector3d RemoveGravity(
        const Eigen::Vector3d &f_b,
        const Eigen::Matrix3d &C_nb
    );

    /**
     * @brief  apply motion constraint on velocity estimation
     * @param  void
     * @return void
     */
    void ApplyMotionConstraint(void);
    
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
     * @brief  get velocity delta
     * @param  index_curr, current imu measurement buffer index
     * @param  index_prev, previous imu measurement buffer index
     * @param  R_curr, corresponding orientation of current imu measurement
     * @param  R_prev, corresponding orientation of previous imu measurement
     * @param  velocity_delta, velocity delta output
     * @param  linear_acc_mid, mid-value unbiased linear acc
     * @return true if success false otherwise
     */
    bool GetVelocityDelta(
        const size_t index_curr, const size_t index_prev,
        const Eigen::Matrix3d &R_curr, const Eigen::Matrix3d &R_prev, 
        double &T, 
        Eigen::Vector3d &velocity_delta
    );
    /**
     * @brief  update orientation with effective velocity change velocity_delta
     * @param  T, timestamp delta 
     * @param  velocity_delta, effective velocity change
     * @return void
     */
    void UpdatePosition(const double &T, const Eigen::Vector3d &velocity_delta);
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
    void CorrectStateEstimationPosi(const Eigen::Matrix4d &T_nb);

    /**
     * @brief  get block matrix for observation by orientation quaternion
     * @param  m_n, measurement in navigation frame
     * @param  q_nb, orientation quaternion
     * @return block matrix Gq
     */
    Eigen::Matrix<double, 3, 4> GetGMOri(
        const Eigen::Vector3d &m_n,
        const Eigen::Quaterniond &q_nb
    );

    /**
     * @brief  correct state estimation using GNSS position and odometer measurement
     * @param  T_nb, input GNSS position 
     * @param  v_b, input odo
     * @return void
     */
    void CorrectStateEstimationPosiVel(
        const Eigen::Matrix4d &T_nb, const Eigen::Vector3d &v_b
    );

    /**
     * @brief  correct state estimation using GNSS position and magneto measurement
     * @param  T_nb, input GNSS position 
     * @param  B_b, input magneto
     * @return void
     */
    void CorrectStateEstimationPosiMag(
        const Eigen::Matrix4d &T_nb, const Eigen::Vector3d &B_b);

    /**
     * @brief  correct state estimation using GNSS position, odometer and magneto measurement
     * @param  T_nb, input GNSS position 
     * @param  v_b, input odo
     * @param  B_b, input magneto
     * @return void
     */
    void CorrectStateEstimationPosiVelMag(
        const Eigen::Matrix4d &T_nb, const Eigen::Vector3d &v_b, const Eigen::Vector3d &B_b
    );

    /**
     * @brief  correct state estimation
     * @param  measurement_type, measurement type
     * @param  measurement, input measurement
     * @return void
     */
    void CorrectStateEstimation(
        const MeasurementType &measurement_type, 
        const Measurement &measurement
    );

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
     * @brief  update observability analysis for GNSS position
     * @param  void
     * @return void
     */
    void UpdateObservabilityAnalysisPosi(
        const double &time, std::vector<double> &record
    );

    /**
     * @brief  update observability analysis for GNSS position & magneto measurement
     * @param  void
     * @return void
     */
    void UpdateObservabilityAnalysisPosiVel(
        const double &time, std::vector<double> &record
    );

    /**
     * @brief  update observability analysis for GNSS position & magneto measurement
     * @param  void
     * @return void
     */
    void UpdateObservabilityAnalysisPosiMag(
        const double &time, std::vector<double> &record
    );

protected:
    KalmanFilter(){}

    // time:
    double time_;

    // data buff:
    std::deque<IMUData> imu_data_buff_;

    // earth constants:
    Eigen::Vector3d g_;
    Eigen::Vector3d w_;
    Eigen::Vector3d b_;

    // observability analysis:
    struct {
        std::vector<double> time_;
        std::vector<Eigen::MatrixXd> Q_;
        
        std::vector<std::vector<double>> pose_;
        std::vector<std::vector<double>> pose_vel_;
        std::vector<std::vector<double>> posi_;
        std::vector<std::vector<double>> posi_vel_;
        std::vector<std::vector<double>> posi_mag_;
        std::vector<std::vector<double>> posi_vel_mag_;
    } observability;

    // hyper-params:
    // a. earth constants:
    struct {
        double GRAVITY_MAGNITUDE;
        double ROTATION_SPEED;
        double LATITUDE;
        double LONGITUDE;
        struct {
            double B_E;
            double B_N;
            double B_U;
        } MAG;
    } EARTH;
    // b. prior state covariance, process & measurement noise:
    struct {
        struct {
            double POSI;
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
            struct {
                double POSI;
                double ORI;
            } POSE;
            double POSI;
            double VEL;
            double ORI;
            double MAG;
        } MEASUREMENT;
    } COV;
    // c. motion constraint:
    struct {
        bool ACTIVATED;
        double W_B_THRESH;
    } MOTION_CONSTRAINT;
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_MODELS_KALMAN_FILTER_KALMAN_FILTER_HPP_