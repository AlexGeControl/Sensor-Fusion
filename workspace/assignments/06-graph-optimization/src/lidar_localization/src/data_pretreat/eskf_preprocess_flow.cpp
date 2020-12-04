/*
 * @Description: IMU/GNSS measurement preprocess for ESKF observability analysis
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */
#include "lidar_localization/data_pretreat/eskf_preprocess_flow.hpp"

#include "glog/logging.h"
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {

ESKFPreprocessFlow::ESKFPreprocessFlow(
    ros::NodeHandle& nh
) {
    // subscribers:
    // a. GNSS-INS-Sim IMU:
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/sim/sensor/imu", 1000000);
    // b. GNSS-INS-Sim GNSS:
    gnss_sub_ptr_ = std::make_shared<GNSSSubscriber>(nh, "/sim/sensor/gps/fix", 1000000);
    velocity_sub_ptr_ = std::make_shared<VelocitySubscriber>(nh, "/sim/sensor/gps/vel", 1000000);
    // c. reference trajectory:
    ref_pose_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/reference_pose", 100000);

    // publishers:
    imu_pub_ptr_ = std::make_shared<IMUPublisher>(nh, "/synced_imu", "/imu_link", 100);
    gnss_pose_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/synced_gnss_pose", "/map", "/imu_link", 100);
    ref_pose_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/synced_reference_pose", "/map", "/imu_link", 100);
}

bool ESKFPreprocessFlow::Run() {
    if (!ReadData())
        return false;
    
    if (!InitGNSS())
        return false;

    while(HasData()) {
        if (!ValidData()) {
            LOG(INFO) << "Invalid data. Skip." << std::endl;
            continue;
        }

        TransformData();
        PublishData();
    }

    return true;
}

bool ESKFPreprocessFlow::ReadData() {
    // fetch IMU measurements from buffer:
    imu_sub_ptr_->ParseData(imu_data_buff_);
    gnss_sub_ptr_->ParseData(gnss_data_buff_);
    velocity_sub_ptr_->ParseData(velocity_data_buff_);
    ref_pose_sub_ptr_->ParseData(ref_pose_data_buff_);

    if (imu_data_buff_.size() == 0)
        return false;

    return true;
}

bool ESKFPreprocessFlow::InitGNSS() {
    static bool gnss_inited = false;

    if ( !gnss_inited ) {
        GNSSData gnss_data = gnss_data_buff_.front();
        gnss_data.InitOriginPosition();
        gnss_inited = true;
    }

    return gnss_inited;
}

bool ESKFPreprocessFlow::HasData() {
    if (
        imu_data_buff_.size() == 0 ||
        gnss_data_buff_.size() == 0 ||
        velocity_data_buff_.size() == 0 ||
        ref_pose_data_buff_.size() == 0 
    ) {
        return false;
    }

    return true;
}

bool ESKFPreprocessFlow::ValidData() {
    current_imu_data_ = imu_data_buff_.front();
    current_gnss_data_ = gnss_data_buff_.front();
    current_velocity_data_ = velocity_data_buff_.front();
    current_ref_pose_data_ = ref_pose_data_buff_.front();

    double diff_gnss_time = current_imu_data_.time - current_gnss_data_.time;
    double diff_velocity_time = current_imu_data_.time - current_velocity_data_.time;
    double diff_ref_pose_time = current_imu_data_.time - current_ref_pose_data_.time;

    //
    // this check assumes the frequency of IMU/GNSS is 100Hz:
    //
    if ( diff_gnss_time < -0.005 || diff_velocity_time < -0.005 || diff_ref_pose_time < -0.005 ) {
        imu_data_buff_.pop_front();
        return false;
    }

    if (diff_gnss_time > 0.005) {
        gnss_data_buff_.pop_front();
        return false;
    }

    if (diff_velocity_time > 0.005) {
        velocity_data_buff_.pop_front();
        return false;
    }

    if (diff_ref_pose_time > 0.005) {
        ref_pose_data_buff_.pop_front();
        return false;
    }

    imu_data_buff_.pop_front();
    gnss_data_buff_.pop_front();
    velocity_data_buff_.pop_front();
    ref_pose_data_buff_.pop_front();

    return true;
}

bool ESKFPreprocessFlow::TransformData() {
    // a. get GNSS position measurement:
    gnss_pose_ = Eigen::Matrix4f::Identity();

    current_gnss_data_.UpdateXYZ();
    
    gnss_pose_(0,3) = current_gnss_data_.local_N;
    gnss_pose_(1,3) = current_gnss_data_.local_E;
    gnss_pose_(2,3) = current_gnss_data_.local_U;

    // b. transform velocity measurement from NED to ENU:
    current_velocity_data_.NED2ENU();

    // c. transform reference pose position from LLA to xyz:
    GNSSData ref_position;

    ref_position.latitude  = current_ref_pose_data_.pose(0, 3);
    ref_position.longitude = current_ref_pose_data_.pose(1, 3);
    ref_position.altitude  = current_ref_pose_data_.pose(2, 3);

    ref_position.UpdateXYZ();

    current_ref_pose_data_.pose(0,3) = ref_position.local_N;
    current_ref_pose_data_.pose(1,3) = ref_position.local_E;
    current_ref_pose_data_.pose(2,3) = ref_position.local_U;

    return true;
}

bool ESKFPreprocessFlow::PublishData() {
    imu_pub_ptr_->Publish(current_imu_data_, current_imu_data_.time);
    gnss_pose_pub_ptr_->Publish(gnss_pose_, current_velocity_data_, current_imu_data_.time);
    ref_pose_pub_ptr_->Publish(
        current_ref_pose_data_.pose, 
        current_ref_pose_data_.vel.v, 
        current_imu_data_.time
    );

    return true;
}

} // namespace lidar_localization