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
    return true;
    
    if (
        kalman_filter_ptr_->Correct(
            imu_data,
            gnss_pose_data.time, KalmanFilter::MeasurementType::POSITION, init_pose_.inverse() * gnss_pose_data.pose
        )
    ) {
        kalman_filter_ptr_->GetOdometry(
            current_pose_, current_vel_
        );
        return true;
    }

    return false;
}

void IMUGNSSFiltering::GetOdometry(Eigen::Matrix4f &pose, Eigen::Vector3f &vel) {
    pose = init_pose_ * current_pose_;
    vel = init_pose_.block<3, 3>(0, 0) * current_vel_;
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