/*
 * @Description: IMU-GNSS fusion for localization workflow
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */
#ifndef LIDAR_LOCALIZATION_FILTERING_IMU_GNSS_FILTERING_FLOW_HPP_
#define LIDAR_LOCALIZATION_FILTERING_IMU_GNSS_FILTERING_FLOW_HPP_

#include <ros/ros.h>

// subscribers:
// a. IMU:
#include "lidar_localization/subscriber/imu_subscriber.hpp"
// b. GNSS & reference pose:
#include "lidar_localization/subscriber/odometry_subscriber.hpp"

// publishers:
#include "lidar_localization/publisher/odometry_publisher.hpp"
#include "lidar_localization/publisher/tf_broadcaster.hpp"

// filtering instance:
#include "lidar_localization/filtering/imu_gnss_filtering.hpp"

#include "glog/logging.h"

namespace lidar_localization {

class IMUGNSSFilteringFlow {
  public:
    IMUGNSSFilteringFlow(ros::NodeHandle& nh);
    bool Run();
    // save odometry for evo evaluation:
    bool SaveOdometry(void);
    bool SaveObservabilityAnalysis(void);

  private:
    bool ReadData();
    bool HasInited();
    
    bool HasData();

    bool HasIMUData(void) { 
      return ( !imu_data_buff_.empty() ); 
    }
    bool HasGNSSData(void) { 
      return ( !gnss_data_buff_.empty() );
    }
    bool HasIMUComesFirst(void) const { return imu_data_buff_.front().time < gnss_data_buff_.front().time; }

    bool ValidIMUData();
    bool ValidGNSSData();

    bool InitLocalization();
    
    bool UpdateLocalization();
    bool CorrectLocalization();

    bool PublishFusionOdom();

    bool UpdateOdometry();
    /**
     * @brief  save pose in KITTI format for evo evaluation
     * @param  pose, input pose
     * @param  ofs, output file stream
     * @return true if success otherwise false
     */
    bool SavePose(
        const Eigen::Matrix4f& pose, 
        std::ofstream& ofs
    );

  private:
    // subscriber:
    // a. IMU:
    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
    std::deque<IMUData> imu_data_buff_;
    // b. GNSS:
    std::shared_ptr<OdometrySubscriber> gnss_sub_ptr_;
    std::deque<PoseData> gnss_data_buff_;
    // c. reference trajectory:
    std::shared_ptr<OdometrySubscriber> ref_pose_sub_ptr_;
    std::deque<PoseData> ref_pose_data_buff_;

    // publisher:
    // a. odometry:
    std::shared_ptr<OdometryPublisher> fused_odom_pub_ptr_;
    // b. tf:
    std::shared_ptr<TFBroadCaster> imu_tf_pub_ptr_;

    // filtering instance:
    std::shared_ptr<IMUGNSSFiltering> filtering_ptr_;

    IMUData current_imu_data_;
    PoseData current_gnss_data_;
    PoseData current_ref_pose_data_;
    
    // lidar odometry frame in map frame:
    Eigen::Matrix4f fused_pose_ = Eigen::Matrix4f::Identity();
    Eigen::Vector3f fused_vel_ = Eigen::Vector3f::Zero();

    // trajectory for evo evaluation:
    struct {
      size_t N = 0;

      std::deque<Eigen::Matrix4f> fused_;
      std::deque<Eigen::Matrix4f> gnss_;
      std::deque<Eigen::Matrix4f> ref_;
    } trajectory;
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_FILTERING_IMU_GNSS_FILTERING_FLOW_HPP_