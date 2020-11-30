/*
 * @Description: IMU pre-integrator for LIO mapping, interface
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 */

#ifndef LIDAR_LOCALIZATION_MODELS_PRE_INTEGRATOR_IMU_PRE_INTEGRATOR_HPP_
#define LIDAR_LOCALIZATION_MODELS_PRE_INTEGRATOR_IMU_PRE_INTEGRATOR_HPP_

#include "lidar_localization/models/pre_integrator/pre_integrator.hpp"

#include "lidar_localization/sensor_data/imu_data.hpp"

#include <sophus/so3.hpp>

namespace lidar_localization {

class IMUPreIntegrator : public PreIntegrator {
public:
    IMUPreIntegrator(void);

    /**
     * @brief  init IMU pre-integrator
     * @param  init_imu_data, init IMU measurements
     * @return true if success false otherwise
     */
    bool Init(const IMUData &init_imu_data);

    /**
     * @brief  update IMU pre-integrator
     * @param  imu_data, current IMU measurements
     * @return true if success false otherwise
     */
    bool Update(const IMUData &init_imu_data);

    /**
     * @brief  get measurement for IMU pre-integration edge
     * @param  void
     * @return true if success false otherwise
     */
    bool GetMeasurement(void);

    /**
     * @brief  get information matrix for IMU pre-integration edge
     * @param  void
     * @return true if success false otherwise
     */
    bool GetInformation(void);

private:
    static const int DIM_STATE = 15;
    static const int DIM_NOISE = 15;

    static const int INDEX_ALPHA = 0;
    static const int INDEX_THETA = 3;
    static const int INDEX_BETA = 6;
    static const int INDEX_B_A = 9;
    static const int INDEX_B_G = 12;

    typedef Eigen::Matrix<double, DIM_STATE, DIM_STATE> MatrixF;
    typedef Eigen::Matrix<double, DIM_STATE, DIM_STATE> MatrixP;
    typedef Eigen::Matrix<double, DIM_STATE, DIM_NOISE> MatrixG;
    typedef Eigen::Matrix<double, DIM_NOISE, DIM_NOISE> MatrixR;

    // data buff:
    std::deque<IMUData> imu_data_buff_;

    // pre-integration state:
    struct {
        Eigen::Vector3d alpha_ij;
        Eigen::Vector3d beta_ij;
        // here works on so3 for simplicity:
        Sophus::SO3d theta_ij;
        Eigen::Vector3d b_a_i;
        Eigen::Vector3d b_g_i;
    } state_;

    // gravity:
    Eigen::Vector3d g_;

    // process equation:
    MatrixF F_ = MatrixF::Zero();
    MatrixG G_ = MatrixG::Zero();
    MatrixP P_ = MatrixP::Zero();
    MatrixR R_ = MatrixR::Zero();

    /**
     * @brief  reset pre-integrator state using IMU measurements
     * @param  void
     * @return void
     */
    void ResetState(const IMUData &init_imu_data);

    /**
     * @brief  update pre-integrator state: mean, covariance and Jacobian
     * @param  void
     * @return void
     */
    void UpdateState(void);
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_MODELS_PRE_INTEGRATOR_IMU_PRE_INTEGRATOR_HPP_