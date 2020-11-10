/*
 * @Description: IMU integration activity
 * @Author: Ge Yao
 * @Date: 2020-11-10 14:25:03
 */
#ifndef IMU_INTEGRATION_ACTIVITY_HPP_
#define IMU_INTEGRATION_ACTIVITY_HPP_

// common:
#include <ros/ros.h>

// subscribers:
#include "imu_integration/subscriber/imu_subscriber.hpp"

namespace imu_integration {

class Activity {
  public:
    Activity(ros::NodeHandle& nh, std::string imu_topic);
    bool Run(void);

  private:
    bool ReadData(void);
    bool HasData(void);
    bool UpdatePose(void);
    bool PublishPose(void);

  private:
    // subscriber:
    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;

    // data buffer:
    std::deque<IMUData> imu_data_buff_;

    // IMU pose estimation:
    Eigen::Matrix4f imu_pose_ = Eigen::Matrix4f::Identity();
};

} // namespace imu_integration

#endif 