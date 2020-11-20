/*
 * @Description: IMU-lidar fusion for localization workflow
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */

#include "lidar_localization/filtering/imu_gnss_filtering_flow.hpp"

#include "lidar_localization/filtering/imu_gnss_filtering.hpp"

#include "lidar_localization/tools/file_manager.hpp"

#include "glog/logging.h"
#include <ostream>
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {

IMUGNSSFilteringFlow::IMUGNSSFilteringFlow(
    ros::NodeHandle& nh
) {
    // subscriber:
    // a. IMU raw measurement:
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/synced_imu", 1000000);
    // b. GNSS raw measurement:
    gnss_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/synced_gnss_pose", 100000);
    // c. reference trajectory:
    ref_pose_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/synced_reference_pose", 100000);
    
    // publisher:
    // a. fused pose in map frame:
    fused_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/fused_pose", "/map", "/imu_link", 100);
    // b. tf, map -> imu_link:
    imu_tf_pub_ptr_ = std::make_shared<TFBroadCaster>("/map", "/imu_link");
    // c. covariance:
    fused_std_pub_ = nh.advertise<lidar_localization::ESKFStd>("/fused_std", 100);
    fused_std_.header.frame_id = "/imu_link";

    // filtering instance:
    filtering_ptr_ = std::make_shared<IMUGNSSFiltering>();
}

bool IMUGNSSFilteringFlow::Run() {
    ReadData();

    while( HasData() ) {
        if ( !HasInited() ) {
            if ( 
                HasGNSSData() && ValidGNSSData() &&
                HasIMUData() && ValidIMUData()
            ) {
                InitLocalization();
            }
        } else {
            // TODO: handle timestamp chaos in an more elegant way
            if (  HasGNSSData() && ValidGNSSData() ) {
                if ( HasIMUData() ) {
                    while (
                        HasIMUData() && ValidIMUData() && 
                        current_imu_data_.time < current_gnss_data_.time
                    ) {
                        UpdateLocalization();
                    }

                    if (
                        current_imu_data_.time >= current_gnss_data_.time
                    ) {
                        gnss_data_buff_.push_back(current_gnss_data_);
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

bool IMUGNSSFilteringFlow::SaveOdometry(void) {
    if ( 0 == trajectory.N ) {
        return false;
    }

    // init output files:
    std::ofstream fused_odom_ofs;
    std::ofstream gnss_odom_ofs;
    std::ofstream ref_odom_ofs;
    if (
        !FileManager::CreateFile(fused_odom_ofs, WORK_SPACE_PATH + "/slam_data/trajectory/fused.txt") ||
        !FileManager::CreateFile(gnss_odom_ofs, WORK_SPACE_PATH + "/slam_data/trajectory/gnss.txt") ||
        !FileManager::CreateFile(ref_odom_ofs, WORK_SPACE_PATH + "/slam_data/trajectory/ground_truth.txt")
    ) {
        return false;
    }

    // write outputs:
    for (size_t i = 0; i < trajectory.N; ++i) {
        SavePose(trajectory.fused_.at(i), fused_odom_ofs);
        SavePose(trajectory.gnss_.at(i), gnss_odom_ofs);

        // sync ref pose with gnss measurement:
        while (
            !ref_pose_data_buff_.empty() && 
            (ref_pose_data_buff_.front().time - trajectory.time_.at(i) <= -0.005)
        ) {
            ref_pose_data_buff_.pop_front();
        }
    
        if ( ref_pose_data_buff_.empty() ) {
            break;
        }
        current_ref_pose_data_ = ref_pose_data_buff_.front();
        
        SavePose(current_ref_pose_data_.pose, ref_odom_ofs);
    }

    return true;
}

bool IMUGNSSFilteringFlow::SaveObservabilityAnalysis(void) {
    filtering_ptr_->SaveObservabilityAnalysis();

    return true;
}

bool IMUGNSSFilteringFlow::ReadData() {
    //
    // pipe synced IMU-GNSS measurements into buffer:
    // 
    imu_sub_ptr_->ParseData(imu_data_buff_);
    gnss_sub_ptr_->ParseData(gnss_data_buff_);
    ref_pose_sub_ptr_->ParseData(ref_pose_data_buff_);

    return true;
}

bool IMUGNSSFilteringFlow::HasInited(void) {
    return filtering_ptr_->HasInited();
}

bool IMUGNSSFilteringFlow::HasData() {
    if ( !HasInited() ) {
        if ( !HasIMUData() || !HasGNSSData() ) {
            return false;
        }
    } else {
        if ( !HasIMUData() && !HasGNSSData() ) {
            return false;
        }
    }

    return true;
}

bool IMUGNSSFilteringFlow::ValidIMUData() {
    current_imu_data_ = imu_data_buff_.front();

    imu_data_buff_.pop_front();

    return true;
}

bool IMUGNSSFilteringFlow::ValidGNSSData() {
    current_gnss_data_ = gnss_data_buff_.front();
    
    gnss_data_buff_.pop_front();

    return true;
}

bool IMUGNSSFilteringFlow::InitLocalization(void) {
    // 
    filtering_ptr_->Init(
        current_gnss_data_.pose,
        current_gnss_data_.vel,
        current_imu_data_
    );
    
    LOG(INFO) << "Init Error-State Kalman Filter with first GNSS measurement" << std::endl;

    return true;
}

bool IMUGNSSFilteringFlow::UpdateLocalization() {
    if ( 
        filtering_ptr_->Update(
            current_imu_data_
        ) 
    ) {
        // publish new odom estimation:
        PublishFusionOdom();

        return true;
    }

    return false;
}

bool IMUGNSSFilteringFlow::CorrectLocalization() {
    static int count = 0;

    if ( 
        // downsample GNSS measurement:
        0 == (++count % 10) && 
        // successful correct:
        filtering_ptr_->Correct(
            current_imu_data_, 
            current_gnss_data_
        ) 
    ) {
        // reset downsample counter:
        count = 0;

        // publish new odom estimation:
        PublishFusionOdom();
        
        // add to odometry output for evo evaluation:
        UpdateOdometry(current_gnss_data_.time);

        return true;
    }

    return false;
}

bool IMUGNSSFilteringFlow::PublishFusionOdom() {
    // get odometry from Kalman filter:
    filtering_ptr_->GetOdometry(fused_pose_, fused_vel_);

    // a. publish fusion odometry:
    fused_odom_pub_ptr_->Publish(fused_pose_, fused_vel_, current_imu_data_.time);
    // b. publish tf:
    imu_tf_pub_ptr_->SendTransform(fused_pose_, current_imu_data_.time);

    // publish standard deviation:
    PublishFusionStandardDeviation();

    return true;
}

bool IMUGNSSFilteringFlow::PublishFusionStandardDeviation() {
    // get standard deviation from Kalman filter:
    filtering_ptr_->GetStandardDeviation(fused_std_);
    // c. publish standard deviation:
    fused_std_pub_.publish(fused_std_);

    return true;
}

bool IMUGNSSFilteringFlow::UpdateOdometry(const double &time) {
    trajectory.time_.push_back(time);

    trajectory.fused_.push_back(fused_pose_);
    trajectory.gnss_.push_back(current_gnss_data_.pose);

    ++trajectory.N;

    return true;
}

/**
 * @brief  save pose in KITTI format for evo evaluation
 * @param  pose, input pose
 * @param  ofs, output file stream
 * @return true if success otherwise false
 */
bool IMUGNSSFilteringFlow::SavePose(
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