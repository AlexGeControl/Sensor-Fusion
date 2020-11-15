/*
 * @Description: IMU-lidar fusion for localization workflow
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */

#include "lidar_localization/global_defination/global_defination.h"

#include "lidar_localization/filtering/filtering_flow.hpp"

#include "lidar_localization/tools/file_manager.hpp"

#include "glog/logging.h"
#include <ostream>


namespace lidar_localization {

FilteringFlow::FilteringFlow(
    ros::NodeHandle& nh
) {
    // subscriber:
    // a. IMU raw measurement:
    imu_raw_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu/extract", 1000000);
    // b. undistorted Velodyne measurement: 
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/synced_cloud", 100000);
    // c. lidar pose in map frame:
    gnss_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/synced_gnss", 100000);
    // d. IMU synced measurement:
    imu_synced_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/synced_imu", 100000); 
    // e. lidar to imu tf:
    lidar_to_imu_ptr_ = std::make_shared<TFListener>(nh, "/imu_link", "/velo_link");
    
    // publisher:
    // a. global point cloud map:
    global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/global_map", "/map", 100);
    // b. local point cloud map:
    local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/local_map", "/map", 100);
    // c. current scan:
    current_scan_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/current_scan", "/map", 100);
    // d. estimated lidar pose in map frame:
    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/laser_localization", "/map", "/lidar", 100);
    // e. fused pose in map frame:
    fused_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/fused_localization", "/map", "/lidar", 100);
    // f. ground truth:
    ref_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/ref_localization", "/map", "/lidar", 100);

    // f. tf:
    laser_tf_pub_ptr_ = std::make_shared<TFBroadCaster>("/map", "/vehicle_link");

    filtering_ptr_ = std::make_shared<Filtering>();
}

bool FilteringFlow::Run() {
    if ( !InitCalibration() ) {
        return false;
    }

    // if new global map is available, publish it:
    PublishGlobalMap();
    // if new local map is available, publish it:
    PublishLocalMap();

    ReadData();

    while( HasData() ) {
        if ( !HasInited() ) {
            if ( ValidLidarData() ) {
                InitLocalization();
            }
        } else {
            // TODO: handle timestamp chaos in an more elegant way
            if (  HasLidarData() && ValidLidarData() ) {
                if ( HasIMUData() ) {
                    while (
                        HasIMUData() && ValidIMUData() && 
                        current_imu_raw_data_.time < current_cloud_data_.time
                    ) {
                        LOG(INFO) << "Update before Correct..." << std::endl;
                        UpdateLocalization();
                    }

                    if (
                        current_imu_raw_data_.time >= current_cloud_data_.time
                    ) {
                        imu_raw_data_buff_.push_back(current_imu_raw_data_);
                    }
                }

                CorrectLocalization();
            }

            if ( HasIMUData() && ValidIMUData() ) {
                UpdateLocalization();
            }
        }
    }

    return true;
}

bool FilteringFlow::SaveOdometry(void) {
    if ( 0 == trajectory.N ) {
        return false;
    }

    // init output files:
    std::ofstream fused_odom_ofs;
    std::ofstream laser_odom_ofs;
    std::ofstream ref_odom_ofs;
    if (
        !FileManager::CreateFile(fused_odom_ofs, WORK_SPACE_PATH + "/slam_data/trajectory/fused.txt") ||
        !FileManager::CreateFile(laser_odom_ofs, WORK_SPACE_PATH + "/slam_data/trajectory/laser.txt") ||
        !FileManager::CreateFile(ref_odom_ofs, WORK_SPACE_PATH + "/slam_data/trajectory/ground_truth.txt")
    ) {
        return false;
    }

    // write outputs:
    for (size_t i = 0; i < trajectory.N; ++i) {
        SavePose(trajectory.fused_.at(i), fused_odom_ofs);
        SavePose(trajectory.lidar_.at(i), laser_odom_ofs);
        SavePose(trajectory.ref_.at(i), ref_odom_ofs);
    }

    return true;
}

bool FilteringFlow::ReadData() {
    //
    // pipe raw IMU measurements into buffer:
    // 
    imu_raw_sub_ptr_->ParseData(imu_raw_data_buff_);
    while (
        HasInited() && 
        HasIMUData() && 
        imu_raw_data_buff_.front().time < filtering_ptr_->GetTime()
    ) {
        imu_raw_data_buff_.pop_front();
    }

    //
    // pipe synced lidar-GNSS-IMU measurements into buffer:
    // 
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    gnss_sub_ptr_->ParseData(gnss_data_buff_);
    imu_synced_sub_ptr_->ParseData(imu_synced_data_buff_);

    return true;
}

bool FilteringFlow::HasInited(void) {
    return filtering_ptr_->HasInited();
}

bool FilteringFlow::HasData() {
    if ( !HasInited() ) {
        if ( !HasLidarData() ) {
            return false;
        }
    } else {
        if ( !HasIMUData() && !HasLidarData() ) {
            return false;
        }
    }

    return true;
}

bool FilteringFlow::ValidIMUData() {
    current_imu_raw_data_ = imu_raw_data_buff_.front();

    imu_raw_data_buff_.pop_front();

    return true;
}

bool FilteringFlow::ValidLidarData() {
    current_cloud_data_ = cloud_data_buff_.front();
    current_gnss_data_ = gnss_data_buff_.front();
    current_imu_synced_data_ = imu_synced_data_buff_.front();

    double diff_gnss_time = current_cloud_data_.time - current_gnss_data_.time;
    double diff_imu_time = current_cloud_data_.time - current_imu_synced_data_.time;

    if (diff_gnss_time < -0.05 || diff_imu_time < -0.05) {
        cloud_data_buff_.pop_front();
        return false;
    }

    if (diff_gnss_time > 0.05) {
        gnss_data_buff_.pop_front();
        return false;
    }

    if (diff_imu_time > 0.05) {
        imu_synced_data_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    gnss_data_buff_.pop_front();
    imu_synced_data_buff_.pop_front();

    return true;
}

bool FilteringFlow::InitCalibration() {
    // lookup imu pose in lidar frame:
    static bool calibration_received = false;

    if (!calibration_received) {
        if (lidar_to_imu_ptr_->LookupData(lidar_to_imu_)) {
            calibration_received = true;
        }
    }

    return calibration_received;
}

bool FilteringFlow::InitLocalization(void) {
    // geo ego vehicle velocity in navigation frame:
    Eigen::Vector3f init_vel = current_gnss_data_.vel;

    // first try to init using scan context query:
    if (
        filtering_ptr_->Init(
            current_cloud_data_,
            init_vel,
            current_imu_synced_data_
        )
    ) {
        Eigen::Matrix4f init_pose = filtering_ptr_->GetPose();

        // evaluate deviation from GNSS/IMU:
        float deviation = (
            init_pose.block<3, 1>(0, 3) - current_gnss_data_.pose.block<3, 1>(0, 3)
        ).norm();

        // prompt:
        LOG(INFO) << "Scan Context Localization Init Succeeded. Deviation between GNSS/IMU: " 
                  << deviation
                  << std::endl;
    } 
    // if failed, fall back to GNSS/IMU init:
    else {
        filtering_ptr_->Init(
            current_gnss_data_.pose,
            init_vel,
            current_imu_synced_data_
        );

        LOG(INFO) << "Scan Context Localization Init Failed. Fallback to GNSS/IMU." 
                  << std::endl;
    }
    
    return true;
}

bool FilteringFlow::UpdateLocalization() {
    if ( filtering_ptr_->Update(current_imu_raw_data_) ) {
        LOG(INFO) << "Update" << std::endl;
        PublishFusionOdom();
        return true;
    }

    return false;
}

bool FilteringFlow::CorrectLocalization() {
    bool is_fusion_succeeded = filtering_ptr_->Correct(
        current_imu_synced_data_, 
        current_cloud_data_, 
        laser_pose_
    );
    PublishLidarOdom();

    if ( is_fusion_succeeded ) {
        LOG(INFO) << "Correct" << std::endl;
        PublishFusionOdom();
        // add to odometry output for evo evaluation:
        UpdateOdometry();
        return true;
    }

    return false;
}

bool FilteringFlow::PublishGlobalMap() {
    if (filtering_ptr_->HasNewGlobalMap() && global_map_pub_ptr_->HasSubscribers()) {
        CloudData::CLOUD_PTR global_map_ptr(new CloudData::CLOUD());
        filtering_ptr_->GetGlobalMap(global_map_ptr);
        global_map_pub_ptr_->Publish(global_map_ptr);

        return true;
    }

    return false;
}

bool FilteringFlow::PublishLocalMap() {
    if (filtering_ptr_->HasNewLocalMap() && local_map_pub_ptr_->HasSubscribers()) {
        local_map_pub_ptr_->Publish(filtering_ptr_->GetLocalMap());

        return true;
    }
    
    return false;
}

bool FilteringFlow::PublishLidarOdom() {
    // a. publish lidar odometry
    laser_odom_pub_ptr_->Publish(laser_pose_, current_cloud_data_.time);
    // b. publish current scan:
    current_scan_pub_ptr_->Publish(filtering_ptr_->GetCurrentScan());
    // c. publish ref odometry:
    ref_odom_pub_ptr_->Publish(current_gnss_data_.pose, current_gnss_data_.vel, current_gnss_data_.time);

    return true;
}

bool FilteringFlow::PublishFusionOdom() {
    // get odometry from Kalman filter:
    filtering_ptr_->GetOdometry(fused_pose_, fused_vel_);
    // a. publish tf:
    laser_tf_pub_ptr_->SendTransform(fused_pose_, current_imu_raw_data_.time);
    // b. publish fusion odometry:
    fused_odom_pub_ptr_->Publish(fused_pose_, fused_vel_, current_imu_raw_data_.time);

    return true;
}

bool FilteringFlow::UpdateOdometry(void) {
    trajectory.fused_.push_back(fused_pose_);
    trajectory.lidar_.push_back(laser_pose_);
    trajectory.ref_.push_back(current_gnss_data_.pose);

    ++trajectory.N;

    return true;
}

/**
 * @brief  save pose in KITTI format for evo evaluation
 * @param  pose, input pose
 * @param  ofs, output file stream
 * @return true if success otherwise false
 */
bool FilteringFlow::SavePose(
    const Eigen::Matrix4f& pose, 
    std::ofstream& ofs
) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            ofs << pose(i, j);
            
            if (i == 2 && j == 3) {
                ofs << std::endl;
            } else {
                ofs << " ";
            }
        }
    }

    return true;
}

}