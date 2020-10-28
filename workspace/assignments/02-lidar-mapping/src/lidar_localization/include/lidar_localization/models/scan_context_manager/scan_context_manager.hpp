/*
 * @Description: loop closure detection using scan context
 * @Author: Ge Yao
 * @Date: 2020-10-28 15:43:03
 */

#ifndef LIDAR_LOCALIZATION_MODELS_SCAN_CONTEXT_MANAGER_HPP_
#define LIDAR_LOCALIZATION_MODELS_SCAN_CONTEXT_MANAGER_HPP_

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "lidar_localization/sensor_data/cloud_data.hpp"

namespace lidar_localization {

class ScanContextManager {
public:
    ScanContextManager(const YAML::Node& node);
    void Update(const CloudData &scan);
    void DetectLoopClosure(void);

private:
    Eigen::MatrixXf GetScanContext(const CloudData &scan);

    /**
     * @brief  get orientation of point measurement 
     * @param  x, x component of point measurement
     * @param  y, y component of point measurement
     * @return point measurement orientation, [0.0f, 360.0f)
     */
    float GetOrientation(const float &x, const float &y);
    /**
     * @brief  convert floating point value to integer index 
     * @param  value, target floating point value 
     * @param  MAX_VALUE, max. floating point value
     * @param  RESOLUTION, resolution
     * @return integer index, {0, ..., RESOLUTION - 1}
     */
    int GetIndex(const float &value, const float &MAX_VALUE, const int RESOLUTION);

    // hyper-params:
    float MAX_RADIUS_;
    float MAX_THETA_;

    int NUM_RINGS_;
    int NUM_SECTORS_; 
};

} // namespace lidar_localization

#endif