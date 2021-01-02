/*
 * @Description: LIO localization backend workflow, interface
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 */
#ifndef LIDAR_LOCALIZATION_MATCHING_BACK_END_SLIDING_WINDOW_FLOW_HPP_
#define LIDAR_LOCALIZATION_MATCHING_BACK_END_SLIDING_WINDOW_FLOW_HPP_

#include <ros/ros.h>

//
// subscribers:
//
// a. lidar scan, key frame measurement:
#include "lidar_localization/subscriber/cloud_subscriber.hpp"
// b. lidar odometry & GNSS position:
#include "lidar_localization/subscriber/odometry_subscriber.hpp"
// c. loop closure detection:
#include "lidar_localization/subscriber/loop_pose_subscriber.hpp"
// d. IMU measurement, for pre-integration:
#include "lidar_localization/subscriber/imu_subscriber.hpp"
// e. odometer measurement, for pre-integration:
#include "lidar_localization/subscriber/velocity_subscriber.hpp"

#include "lidar_localization/publisher/odometry_publisher.hpp"
#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "lidar_localization/publisher/key_frame_publisher.hpp"
#include "lidar_localization/publisher/key_frames_publisher.hpp"

#include "lidar_localization/mapping/back_end/lio_mapping.hpp"

namespace lidar_localization {

class SlidingWindowFlow {
public:
    SlidingWindowFlow(
      ros::NodeHandle& nh
    );

    bool Run();
    bool ForceOptimize();
    bool SaveOptimizedOdometry();
    
  private:
    bool ReadData();
    bool HasData();
    bool ValidData();
    bool UpdateIMUPreIntegration(void);
    bool UpdateBackEnd();
    bool PublishData();

  private:
    //
    // subscribers:
    //
    // a. lidar odometry:
    std::shared_ptr<OdometrySubscriber> laser_odom_sub_ptr_;
    std::deque<PoseData> laser_odom_data_buff_;
    // b. map matching odometry:
    std::shared_ptr<OdometrySubscriber> map_matching_odom_sub_ptr_;
    std::deque<PoseData> map_matching_odom_data_buff_;
    // c. IMU measurement, for pre-integration:
    std::shared_ptr<IMUSubscriber> imu_raw_sub_ptr_;
    std::deque<IMUData> imu_raw_data_buff_;
    std::shared_ptr<IMUSubscriber> imu_synced_sub_ptr_;
    std::deque<IMUData> imu_synced_data_buff_;
    // d. GNSS position:
    std::shared_ptr<OdometrySubscriber> gnss_pose_sub_ptr_;
    std::deque<PoseData> gnss_pose_data_buff_;

    std::shared_ptr<OdometryPublisher> transformed_odom_pub_ptr_;
    std::shared_ptr<CloudPublisher> key_scan_pub_ptr_;
    std::shared_ptr<KeyFramePublisher> key_frame_pub_ptr_;
    std::shared_ptr<KeyFramePublisher> key_gnss_pub_ptr_;
    std::shared_ptr<KeyFramesPublisher> key_frames_pub_ptr_;
    std::shared_ptr<LIOBackEnd> back_end_ptr_;

    PoseData current_laser_odom_data_;
    PoseData current_map_matching_odom_data_;
    PoseData current_gnss_pose_data_;
    IMUData current_imu_data_;
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_MATCHING_BACK_END_SLIDING_WINDOW_FLOW_HPP_