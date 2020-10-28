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

    float GetOrientation(
        const float &x, 
        const float &y
    );
    int GetIndex(
        const float &value, 
        const float &MAX_VALUE, 
        const int RESOLUTION
    );

    // hyper-params:
    float MAX_RADIUS_;
    float MAX_THETA_;

    int NUM_RINGS_;
    int NUM_SECTORS_; 
};

} // namespace lidar_localization

#endif