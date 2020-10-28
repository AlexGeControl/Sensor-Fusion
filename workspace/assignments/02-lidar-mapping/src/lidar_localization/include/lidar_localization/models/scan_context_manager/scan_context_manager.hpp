/*
 * @Description: loop closure detection using scan context
 * @Author: Ge Yao
 * @Date: 2020-10-28 15:43:03
 */

#ifndef LIDAR_LOCALIZATION_MODELS_SCAN_CONTEXT_MANAGER_HPP_
#define LIDAR_LOCALIZATION_MODELS_SCAN_CONTEXT_MANAGER_HPP_

#include <Eigen/Dense>

#include "lidar_localization/sensor_data/cloud_data.hpp"

namespace lidar_localization {

class ScanContextManager {
public:
    void Update(const CloudData &scan);
    void DetectLoopClosure(void);

private:
    Eigen::MatrixXd GetScanContext(const CloudData &scan);

    double GetOrientation(
        const double &x, 
        const double &y
    );
    int GetIndex(
        const double &value, 
        const double &MAX_VALUE, 
        const int RESOLUTION
    );

    // TODO: migrate params below to configuration:
    const double MAX_RADIUS = 80.0;
    const double MAX_THETA = 360.0;

    const int NUM_RINGS = 20;
    const int NUM_SECTORS = 60; 
};

} // namespace lidar_localization

#endif