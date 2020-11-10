/*
 * @Description: IMU integration activity
 * @Author: Ge Yao
 * @Date: 2020-11-10 14:25:03
 */
#ifndef IMU_INTEGRATION_ACTIVITY_HPP_
#define IMU_INTEGRATION_ACTIVITY_HPP_

// common:
#include <ros/ros.h>

// config:
#include "imu_integration/config/config.hpp"

// subscribers:
#include "imu_integration/subscriber/imu_subscriber.hpp"
#include "imu_integration/subscriber/odom_subscriber.hpp"

#include <nav_msgs/Odometry.h>

namespace imu_integration {

namespace estimator {

class Activity {
  public:
    Activity(void);
    void Init(void);
    bool Run(void);
  private:
    bool ReadData(void);
    bool HasData(void);
    bool UpdatePose(void);
    bool PublishPose(void);

  private:
    // node handler:
    ros::NodeHandle private_nh_;

    // subscriber:
    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
    std::shared_ptr<OdomSubscriber> odom_ground_truth_sub_ptr;
    ros::Publisher odom_estimation_pub_;

    // data buffer:
    std::deque<IMUData> imu_data_buff_;
    std::deque<OdomData> odom_data_buff_;

    // config:
    bool initialized_ = false;

    IMUConfig imu_config_;
    OdomConfig odom_config_;

    // a. gravity constant:
    Eigen::Vector3d G_;
    // b. angular velocity:
    Eigen::Vector3d angular_vel_bias_;
    // c. linear acceleration:
    Eigen::Vector3d linear_acc_bias_;

    // IMU pose estimation:
    Eigen::Matrix4d pose_ = Eigen::Matrix4d::Identity();
    Eigen::Vector3d vel_ = Eigen::Vector3d::Zero();

    nav_msgs::Odometry message_odom_;
};

} // namespace estimator

} // namespace imu_integration

#endif 