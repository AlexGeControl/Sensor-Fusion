/*
 * @Description: 地图匹配任务管理， 放在类里使代码更清晰
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:38:42
 */
#include "lidar_localization/matching/matching_flow.hpp"
#include "glog/logging.h"
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {
MatchingFlow::MatchingFlow(ros::NodeHandle& nh) {
    // subscriber:
    // a. undistorted Velodyne measurement: 
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/synced_cloud", 100000);
    // b. lidar pose in map frame:
    gnss_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/synced_gnss", 100000);

    // publisher:
    // a. global point cloud map:
    global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/global_map", "/map", 100);
    // b. local point cloud map:
    local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/local_map", "/map", 100);
    // c. current scan:
    current_scan_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/current_scan", "/map", 100);
    // d. estimated lidar pose in map frame:
    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/laser_localization", "/map", "/lidar", 100);
    laser_tf_pub_ptr_ = std::make_shared<TFBroadCaster>("/map", "/vehicle_link");

    matching_ptr_ = std::make_shared<Matching>();
}

bool MatchingFlow::Run() {
    if (matching_ptr_->HasNewGlobalMap() && global_map_pub_ptr_->HasSubscribers()) {
        CloudData::CLOUD_PTR global_map_ptr(new CloudData::CLOUD());
        matching_ptr_->GetGlobalMap(global_map_ptr);
        global_map_pub_ptr_->Publish(global_map_ptr);
    }

    if (matching_ptr_->HasNewLocalMap() && local_map_pub_ptr_->HasSubscribers())
        local_map_pub_ptr_->Publish(matching_ptr_->GetLocalMap());

    ReadData();

    while(HasData()) {
        if (!ValidData()) {
            LOG(INFO) << "Invalid data. Skip matching" << std::endl;
            continue;
        }

        if (UpdateMatching()) {
            PublishData();
        }
    }

    return true;
}

bool MatchingFlow::ReadData() {
    // pipe lidar measurements and pose into buffer:
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    gnss_sub_ptr_->ParseData(gnss_data_buff_);
    return true;
}

bool MatchingFlow::HasData() {
    if (cloud_data_buff_.size() == 0)
        return false;
    
    if (matching_ptr_->HasInited())
        return true;
    
    if (gnss_data_buff_.size() == 0)
        return false;
        
    return true;
}

bool MatchingFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();

    if (matching_ptr_->HasInited()) {
        cloud_data_buff_.pop_front();
        gnss_data_buff_.clear();
        return true;
    }

    current_gnss_data_ = gnss_data_buff_.front();

    double diff_time = current_cloud_data_.time - current_gnss_data_.time;
    if (diff_time < -0.05) {
        cloud_data_buff_.pop_front();
        return false;
    }

    if (diff_time > 0.05) {
        gnss_data_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    gnss_data_buff_.pop_front();

    return true;
}

bool MatchingFlow::UpdateMatching() {
    if (!matching_ptr_->HasInited()) {
        if (
            matching_ptr_->SetScanContextPose(current_cloud_data_)
        ) {
            Eigen::Matrix4f init_pose = matching_ptr_->GetInitPose();

            // evaluate deviation from GNSS/IMU:
            float deviation = (
                init_pose.block<3, 1>(0, 3) - current_gnss_data_.pose.block<3, 1>(0, 3)
            ).norm();

            // prompt:
            LOG(INFO) << "Scan Context Localization Init Succeeded. Deviation between GNSS/IMU: " 
                      << deviation
                      << std::endl;
        } else {
            matching_ptr_->SetGNSSPose(current_gnss_data_.pose);

            LOG(INFO) << "Scan Context Localization Init Failed. Fallback to GNSS/IMU." 
                      << std::endl;
        }
    }

    return matching_ptr_->Update(current_cloud_data_, laser_odometry_);
}

bool MatchingFlow::PublishData() {
    laser_tf_pub_ptr_->SendTransform(laser_odometry_, current_cloud_data_.time);
    laser_odom_pub_ptr_->Publish(laser_odometry_, current_cloud_data_.time);
    current_scan_pub_ptr_->Publish(matching_ptr_->GetCurrentScan());

    return true;
}
}