/*
 * @Description: IMU pre-integrator for LIO mapping, interface
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 */

#ifndef LIDAR_LOCALIZATION_MODELS_PRE_INTEGRATOR_IMU_PRE_INTEGRATOR_HPP_
#define LIDAR_LOCALIZATION_MODELS_PRE_INTEGRATOR_IMU_PRE_INTEGRATOR_HPP_

#include "lidar_localization/models/pre_integrator/pre_integrator.hpp"

#include "lidar_localization/sensor_data/imu_data.hpp"

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
    // data buff:
    std::deque<IMUData> imu_data_buff_;
    // pre-integration state:
    struct {
        Eigen::Vector3d alpha_ij;
        Eigen::Vector3d beta_ij;
        Eigen::Quaterniond q_ij;
        Eigen::Vector3d b_a_i;
        Eigen::Vector3d b_g_i;
    } state_;

    /**
     * @brief  reset pre-integrator state using IMU measurements
     * @param  void
     * @return void
     */
    void ResetState(const IMUData &init_imu_data);

    /**
     * @brief  update pre-integrator state, mean
     * @param  void
     * @return void
     */
    void UpdateStateMean(void);

    /**
     * @brief  update pre-integrator state, covariance
     * @param  void
     * @return void
     */
    void UpdateStateCovariance(void);
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_MODELS_PRE_INTEGRATOR_IMU_PRE_INTEGRATOR_HPP_