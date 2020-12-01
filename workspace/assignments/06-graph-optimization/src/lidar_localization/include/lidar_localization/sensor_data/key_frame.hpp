/*
 * @Description: LIO key frame
 * @Author: Ren Qian
 * @Date: 2020-02-28 19:13:26
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_KEY_FRAME_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_KEY_FRAME_HPP_

#include <Eigen/Dense>

namespace lidar_localization {
class KeyFrame {
  public:
    double time = 0.0;

    // key frame ID:
    unsigned int index = 0;
    
    // a. position & orientation:
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    // b. velocity:
    Eigen::Vector3f vel = Eigen::Vector3f::Zero();
    // c. bias:
    struct {
      // c.1. accelerometer:
      Eigen::Vector3f accel = Eigen::Vector3f::Zero();
      // c.2. gyroscope:
      Eigen::Vector3f gyro = Eigen::Vector3f::Zero();
    } bias;

  public:
    Eigen::Quaternionf GetQuaternion() const;
    Eigen::Vector3f GetTranslation() const;
};
}

#endif