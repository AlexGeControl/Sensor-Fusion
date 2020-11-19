/*
 * @Description: IMU-lidar fusion for localization workflow
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */
#include "lidar_localization/filtering/imu_gnss_filtering.hpp"

#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"


namespace lidar_localization {

IMUGNSSFiltering::IMUGNSSFiltering() {   
    // load ROS config:
    InitWithConfig();
}

bool IMUGNSSFiltering::Init(
    const Eigen::Matrix4f& init_pose,
    const Eigen::Vector3f &init_vel,
    const IMUData &init_imu_data
) {
    if ( SetInitGNSS(init_pose) ) {
        current_vel_ = init_vel;

        kalman_filter_ptr_->Init(
            current_vel_.cast<double>(),
            init_imu_data
        );
        
        return true;
    }

    return false;
}

bool IMUGNSSFiltering::Update(
    const IMUData &imu_data
) {
    if ( kalman_filter_ptr_->Update(imu_data) ) {
        kalman_filter_ptr_->GetOdometry(
            current_pose_, current_vel_
        );
        return true;
    }

    return false;
}

bool IMUGNSSFiltering::Correct(
    const IMUData &imu_data,
    const PoseData &gnss_pose_data
) {
    static int count = 0;

    if (
        kalman_filter_ptr_->Correct(
            imu_data,
            gnss_pose_data.time, 
            KalmanFilter::MeasurementType::POSITION, init_pose_.inverse() * gnss_pose_data.pose
        )
    ) {
        kalman_filter_ptr_->GetOdometry(
            current_pose_, current_vel_
        );
        
        // downsample for observability analysis:
        if ( 0 == (++count % 10) ) {
            // reset downsample counter:
            count = 0;

            // perform observability analysis:
            kalman_filter_ptr_->UpdateObservabilityAnalysis(
                gnss_pose_data.time,
                KalmanFilter::MeasurementType::POSITION
            );
        }
        
        return true;
    }

    return false;
}

void IMUGNSSFiltering::GetOdometry(Eigen::Matrix4f &pose, Eigen::Vector3f &vel) {
    pose = init_pose_ * current_pose_;
    vel = init_pose_.block<3, 3>(0, 0) * current_vel_;
}

void IMUGNSSFiltering::GetStandardDeviation(ESKFStd &eskf_std_msg) {
    kalman_filter_ptr_->GetCovariance(current_cov_);

    eskf_std_msg.header.stamp = ros::Time(kalman_filter_ptr_->GetTime());

    eskf_std_msg.delta_pos_x_std = std::sqrt(current_cov_.delta_pos.x);
    eskf_std_msg.delta_pos_y_std = std::sqrt(current_cov_.delta_pos.y);
    eskf_std_msg.delta_pos_z_std = std::sqrt(current_cov_.delta_pos.z);

    eskf_std_msg.delta_vel_x_std = std::sqrt(current_cov_.delta_vel.x);
    eskf_std_msg.delta_vel_y_std = std::sqrt(current_cov_.delta_vel.y);
    eskf_std_msg.delta_vel_z_std = std::sqrt(current_cov_.delta_vel.z);

    eskf_std_msg.delta_ori_x_std = std::sqrt(current_cov_.delta_ori.x);
    eskf_std_msg.delta_ori_y_std = std::sqrt(current_cov_.delta_ori.y);
    eskf_std_msg.delta_ori_z_std = std::sqrt(current_cov_.delta_ori.z);

    eskf_std_msg.gyro_bias_x_std = std::sqrt(current_cov_.gyro_bias.x);
    eskf_std_msg.gyro_bias_y_std = std::sqrt(current_cov_.gyro_bias.y);
    eskf_std_msg.gyro_bias_z_std = std::sqrt(current_cov_.gyro_bias.z);

    eskf_std_msg.accel_bias_x_std = std::sqrt(current_cov_.accel_bias.x);
    eskf_std_msg.accel_bias_y_std = std::sqrt(current_cov_.accel_bias.y);
    eskf_std_msg.accel_bias_z_std = std::sqrt(current_cov_.accel_bias.z);
}

void IMUGNSSFiltering::SaveObservabilityAnalysis(void) {
    kalman_filter_ptr_->SaveObservabilityAnalysis(
        KalmanFilter::MeasurementType::POSITION
    );
}

bool IMUGNSSFiltering::InitWithConfig(void) {
    std::string config_file_path = WORK_SPACE_PATH + "/config/filtering/imu_gnss_filtering.yaml";

    YAML::Node config_node = YAML::LoadFile(config_file_path);

    LOG(INFO) << std::endl
              << "-----------------Init IMU-GNSS Fusion for Localization-------------------" 
              << std::endl;
    
    // a. init fusion:
    InitFusion(config_node);

    return true;
}

bool IMUGNSSFiltering::InitFusion(const YAML::Node& config_node) {
    std::string fusion_method = config_node["fusion_method"].as<std::string>();

    std::cout << "\tIMU-GNSS Fusion Method: " << fusion_method << std::endl;

    if (fusion_method == "kalman_filter") {
        kalman_filter_ptr_ = std::make_shared<KalmanFilter>(config_node[fusion_method]);
    } else {
        LOG(ERROR) << "Fusion method " << fusion_method << " NOT FOUND!";
        return false;
    }

    return true;
}

/**
 * @brief  set init pose using GNSS measurement
 * @param  init_scan, init key scan
 * @return true if success otherwise false
 */
bool IMUGNSSFiltering::SetInitGNSS(const Eigen::Matrix4f& gnss_pose) {
    SetInitPose(gnss_pose);
    has_inited_ = true;

    return true;
}

bool IMUGNSSFiltering::SetInitPose(const Eigen::Matrix4f& init_pose) {
    init_pose_ = init_pose;

    return true;
}

} // namespace lidar_localization