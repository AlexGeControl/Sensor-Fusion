/*
 * @Description: loop closure detection using scan context
 * @Author: Ge Yao
 * @Date: 2020-10-28 15:43:03
 */
#include <limits>

#include <math.h>
#include <cmath>

#include "lidar_localization/models/scan_context_manager/scan_context_manager.hpp"
#include "glog/logging.h"

namespace lidar_localization {

void ScanContextManager::Update(const CloudData &scan) {

}

/**
 * @brief  get scan context of given lidar scan
 * @param  scan, lidar scan of key frame
 * @return scan context as Eigen::MatrixXd
 */
Eigen::MatrixXd ScanContextManager::GetScanContext(const CloudData &scan) {
    // num. of point measurements in current scan:
    const size_t N = scan.cloud_ptr->points.size();
    
    // init scan context:
    const double UNKNOWN_HEIGHT = std::numeric_limits<double>::lowest();
    Eigen::MatrixXd scan_context = UNKNOWN_HEIGHT * MatrixXd::Ones(NUM_RINGS, NUM_SECTORS);

    // iterate through point measurements and create scan context:
    double x, y, z;
    double radius, theta;
    for (size_t i = 0; i < N; ++i) {
        // parse point measurement:
        x = scan.cloud_ptr->points.at(i).x;
        y = scan.cloud_ptr->points.at(i).y;
        z = scan.cloud_ptr->points.at(i).z;

        radius = hypot(x, y);
        theta = GetOrientation(x, y);

        // ROI check:
        if (radius > MAX_RADIUS) {
            continue;
        }
        
        // get ring-sector index:
        int rid = GetIndex(radius, MAX_RADIUS, NUM_RINGS); 
        int sid = GetIndex(theta, MAX_THETA, NUM_SECTORS); 

        // update bin height:
        if (scan_context(rid, sid) < z) {
            scan_context(rid, sid) = z;
        }
    }

    // reset unknown height to 0.0 for later cosine distance calculation:
    for (size_t rid = 0; rid < scan_context.rows(); ++rid) {
        for (size_t sid = 0; sid < scan_context.cols(); ++sid) {
            if (UNKNOWN_HEIGHT == scan_context(rid, sid)) {
                scan_context(rid, sid) = 0.0;
            }
        }
    }

    return scan_context;
}

/**
 * @brief  get orientation of point measurement 
 * @param  x, x component of point measurement
 * @param  y, y component of point measurement
 * @return point measurement orientation, [0.0f, 360.0f)
 */
double ScanContextManager::GetOrientation(
    const double &x, 
    const double &y
) {
    double theta = 180.0 / M_PI * atan2(y, x);

    // make sure the orientation is consistent with scan context convension:
    if (theta < 0.0) {
        theta += 360.0;
    }

    return theta;
}

/**
 * @brief  convert floating point value to integer index 
 * @param  value, target floating point value 
 * @param  MAX_VALUE, max. floating point value
 * @param  RESOLUTION, resolution
 * @return integer index, {0, ..., RESOLUTION - 1}
 */
int ScanContextManager::GetIndex(
    const double &value, 
    const double &MAX_VALUE, 
    const int RESOLUTION
) {
    int index = std::floor(static_cast<int>(RESOLUTION*value/MAX_VALUE));

    // this ensures value at MAX_VALUE will be cast into last bin:
    index = std::max(index, RESOLUTION - 1);

    return index;
} 

} // namespace lidar_localization