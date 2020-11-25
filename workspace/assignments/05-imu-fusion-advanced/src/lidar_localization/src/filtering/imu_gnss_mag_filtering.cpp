/*
 * @Description: IMU-GNSS-Mag fusion for localization workflow
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */

#include "lidar_localization/filtering/imu_gnss_mag_filtering.hpp"

#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"


namespace lidar_localization {

IMUGNSSMagFiltering::IMUGNSSMagFiltering() {   
    // load ROS config:
    InitWithConfig();
}

bool IMUGNSSMagFiltering::Init(
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

bool IMUGNSSMagFiltering::Update(
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

bool IMUGNSSMagFiltering::Correct(
    const IMUData &imu_data,
    const PosVelData &pos_vel_data
) {
    static int count = 0;
    
    // set GNSS-odo measurement:
    current_measurement_.time = pos_vel_data.time;

    current_measurement_.T_nb = Eigen::Matrix4d::Identity();
    current_measurement_.T_nb(0, 3) = static_cast<double>(pos_vel_data.pos.x());
    current_measurement_.T_nb(1, 3) = static_cast<double>(pos_vel_data.pos.y());
    current_measurement_.T_nb(2, 3) = static_cast<double>(pos_vel_data.pos.z());    
    current_measurement_.T_nb = init_pose_.inverse().cast<double>() * current_measurement_.T_nb;

    current_measurement_.v_b = pos_vel_data.vel.cast<double>();

    if (
        kalman_filter_ptr_->Correct(
            imu_data,
            ExtendedKalmanFilter::MeasurementType::POSI_VEL, current_measurement_
        )
    ) {
        kalman_filter_ptr_->GetOdometry(
            current_pose_, current_vel_
        );
        
        // downsample for observability analysis:
        if ( 0 == (++count % 10) ) {
            // reset downsample counter:
            count = 0;

            /*
            // perform observability analysis:
            kalman_filter_ptr_->UpdateObservabilityAnalysis(
                gnss_pose_data.time,
                ExtendedKalmanFilter::MeasurementType::POSITION
            );
            */
        }

        return true;
    }

    return false;
}

void IMUGNSSMagFiltering::GetOdometry(Eigen::Matrix4f &pose, Eigen::Vector3f &vel) {
    pose = init_pose_ * current_pose_;
    vel = init_pose_.block<3, 3>(0, 0) * current_vel_;
}

void IMUGNSSMagFiltering::GetStandardDeviation(EKFStd &ekf_std_msg) {
    kalman_filter_ptr_->GetCovariance(current_cov_);

    ekf_std_msg.header.stamp = ros::Time(kalman_filter_ptr_->GetTime());

    ekf_std_msg.pos_x_std = std::sqrt(current_cov_.pos.x);
    ekf_std_msg.pos_y_std = std::sqrt(current_cov_.pos.y);
    ekf_std_msg.pos_z_std = std::sqrt(current_cov_.pos.z);

    ekf_std_msg.vel_x_std = std::sqrt(current_cov_.vel.x);
    ekf_std_msg.vel_y_std = std::sqrt(current_cov_.vel.y);
    ekf_std_msg.vel_z_std = std::sqrt(current_cov_.vel.z);

    ekf_std_msg.ori_w_std = std::sqrt(current_cov_.ori.w);
    ekf_std_msg.ori_x_std = std::sqrt(current_cov_.ori.x);
    ekf_std_msg.ori_y_std = std::sqrt(current_cov_.ori.y);
    ekf_std_msg.ori_z_std = std::sqrt(current_cov_.ori.z);

    ekf_std_msg.gyro_bias_x_std = std::sqrt(current_cov_.gyro_bias.x);
    ekf_std_msg.gyro_bias_y_std = std::sqrt(current_cov_.gyro_bias.y);
    ekf_std_msg.gyro_bias_z_std = std::sqrt(current_cov_.gyro_bias.z);

    ekf_std_msg.accel_bias_x_std = std::sqrt(current_cov_.accel_bias.x);
    ekf_std_msg.accel_bias_y_std = std::sqrt(current_cov_.accel_bias.y);
    ekf_std_msg.accel_bias_z_std = std::sqrt(current_cov_.accel_bias.z);
}

void IMUGNSSMagFiltering::SaveObservabilityAnalysis(void) {
    kalman_filter_ptr_->SaveObservabilityAnalysis(
        ExtendedKalmanFilter::MeasurementType::POSI_VEL
    );
}

bool IMUGNSSMagFiltering::InitWithConfig(void) {
    std::string config_file_path = WORK_SPACE_PATH + "/config/filtering/imu_gnss_mag_filtering.yaml";

    YAML::Node config_node = YAML::LoadFile(config_file_path);

    LOG(INFO) << std::endl
              << "-----------------Init IMU-GNSS-Odo Fusion for Localization-------------------" 
              << std::endl;
    
    // a. init fusion:
    InitFusion(config_node);

    return true;
}

bool IMUGNSSMagFiltering::InitFusion(const YAML::Node& config_node) {
    std::string fusion_method = config_node["fusion_method"].as<std::string>();

    std::cout << "\tIMU-GNSS-Odo Fusion Method: " << fusion_method << std::endl;

    if (fusion_method == "extended_kalman_filter") {
        kalman_filter_ptr_ = std::make_shared<ExtendedKalmanFilter>(config_node[fusion_method]);
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
bool IMUGNSSMagFiltering::SetInitGNSS(const Eigen::Matrix4f& gnss_pose) {
    SetInitPose(gnss_pose);
    has_inited_ = true;

    return true;
}

bool IMUGNSSMagFiltering::SetInitPose(const Eigen::Matrix4f& init_pose) {
    init_pose_ = init_pose;

    return true;
}

} // namespace lidar_localization