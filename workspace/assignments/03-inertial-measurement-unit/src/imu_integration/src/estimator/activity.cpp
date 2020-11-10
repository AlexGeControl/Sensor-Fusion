/*
 * @Description: IMU integration activity
 * @Author: Ge Yao
 * @Date: 2020-11-10 14:25:03
 */
#include "imu_integration/estimator/activity.hpp"
#include "glog/logging.h"

namespace imu_integration {

namespace estimator {

Activity::Activity(void) 
    : private_nh_("~"), 
    initialized_(false),
    // gravity acceleration:
    G_(0, 0, -9.81),
    // angular velocity bias:
    angular_vel_bias_(0.0, 0.0, 0.0),
    // linear acceleration bias:
    linear_acc_bias_(0.0, 0.0, 0.0)
{}

void Activity::Init(void) {
    // parse IMU config:
    private_nh_.param("imu/topic_name", imu_config_.topic_name, std::string("/sim/sensor/imu"));
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(private_nh_, imu_config_.topic_name, 1000000);

    // a. gravity constant:
    private_nh_.param("imu/gravity/x", imu_config_.gravity.x,  0.0);
    private_nh_.param("imu/gravity/y", imu_config_.gravity.y,  0.0);
    private_nh_.param("imu/gravity/z", imu_config_.gravity.z, -9.81);
    G_.x() = imu_config_.gravity.x;
    G_.y() = imu_config_.gravity.y;
    G_.z() = imu_config_.gravity.z;

    // b. angular velocity bias:
    private_nh_.param("imu/bias/angular_velocity/x", imu_config_.bias.angular_velocity.x,  0.0);
    private_nh_.param("imu/bias/angular_velocity/y", imu_config_.bias.angular_velocity.y,  0.0);
    private_nh_.param("imu/bias/angular_velocity/z", imu_config_.bias.angular_velocity.z,  0.0);
    angular_vel_bias_.x() = imu_config_.bias.angular_velocity.x;
    angular_vel_bias_.y() = imu_config_.bias.angular_velocity.y;
    angular_vel_bias_.z() = imu_config_.bias.angular_velocity.z;

    // c. linear acceleration bias:
    private_nh_.param("imu/bias/linear_acceleration/x", imu_config_.bias.linear_acceleration.x,  0.0);
    private_nh_.param("imu/bias/linear_acceleration/y", imu_config_.bias.linear_acceleration.y,  0.0);
    private_nh_.param("imu/bias/linear_acceleration/z", imu_config_.bias.linear_acceleration.z,  0.0);
    linear_acc_bias_.x() = imu_config_.bias.linear_acceleration.x;
    linear_acc_bias_.y() = imu_config_.bias.linear_acceleration.y;
    linear_acc_bias_.z() = imu_config_.bias.linear_acceleration.z;

    // parse odom config:
    private_nh_.param("pose/frame_id", odom_config_.frame_id, std::string("inertial"));
    private_nh_.param("pose/topic_name/ground_truth", odom_config_.topic_name.ground_truth, std::string("/pose/ground_truth"));
    private_nh_.param("pose/topic_name/estimation", odom_config_.topic_name.estimation, std::string("/pose/estimation"));

    odom_ground_truth_sub_ptr = std::make_shared<OdomSubscriber>(private_nh_, odom_config_.topic_name.ground_truth, 1000000);
    odom_estimation_pub_ = private_nh_.advertise<nav_msgs::Odometry>(odom_config_.topic_name.estimation, 500);
}

bool Activity::Run(void) {
    if (!ReadData())
        return false;

    while(HasData()) {
        if (UpdatePose()) {
            PublishPose();
        }
    }

    return true;
}

bool Activity::ReadData(void) {
    // fetch IMU measurements into buffer:
    imu_sub_ptr_->ParseData(imu_data_buff_);

    if (static_cast<size_t>(0) == imu_data_buff_.size())
        return false;

    if (!initialized_) {
        odom_ground_truth_sub_ptr->ParseData(odom_data_buff_);

        if (static_cast<size_t>(0) == odom_data_buff_.size())
            return false;
    }

    return true;
}

bool Activity::HasData(void) {
    if (static_cast<size_t>(0) == imu_data_buff_.size())
        return false;

    if (
        !initialized_ && 
        static_cast<size_t>(0) == odom_data_buff_.size()
    ) {
        return false;
    }

    return true;
}

bool Activity::UpdatePose(void) {
    static double time_prev;
    static Eigen::Vector3d angular_vel_prev;
    static Eigen::Vector3d linear_acc_prev;

    if (!initialized_) {
        OdomData &odom_data = odom_data_buff_.back();
        IMUData &imu_data = imu_data_buff_.back();

        pose_ = odom_data.pose;
        vel_ = odom_data.vel;

        time_prev = imu_data.time;
        // angular velocity should be in body frame:
        angular_vel_prev = imu_data.angular_velocity - angular_vel_bias_;
        // linear acceleration should be in ENU frame:
        Eigen::Matrix3d R = pose_.block<3, 3>(0, 0);
        linear_acc_prev = R*(imu_data.linear_acceleration - linear_acc_bias_) - G_;

        initialized_ = true;

        odom_data_buff_.clear();
        imu_data_buff_.clear();
    } else {
        IMUData &imu_data = imu_data_buff_.front();

        // get time delta:
        double time_curr = imu_data.time;
        double delta_t = time_curr - time_prev;

        // update orientation:
        Eigen::Matrix3d R = pose_.block<3, 3>(0, 0);
        Eigen::Vector3d angular_vel_curr = imu_data.angular_velocity - angular_vel_bias_;
        Eigen::Vector3d angular_vel_mid_value = 0.5*(angular_vel_prev + angular_vel_curr);

        Eigen::Vector3d da = 0.5*delta_t*angular_vel_mid_value;
        Eigen::Quaterniond dq(1.0, da.x(), da.y(), da.z());
        Eigen::Quaterniond q(R);
        q = q*dq;
        pose_.block<3, 3>(0, 0) = R = q.normalized().toRotationMatrix();

        // update position:
        Eigen::Vector3d t = pose_.block<3, 1>(0, 3);
        Eigen::Vector3d linear_acc_curr = R*(imu_data.linear_acceleration - linear_acc_bias_) - G_;
        Eigen::Vector3d linear_acc_mid_value = 0.5*(linear_acc_prev + linear_acc_curr);

        pose_.block<3, 1>(0, 3) = t + delta_t*vel_ + 0.5*delta_t*delta_t*linear_acc_mid_value;
        vel_ = vel_ + delta_t*linear_acc_mid_value;

        // move forward:
        time_prev = time_curr;
        angular_vel_prev = angular_vel_curr;
        linear_acc_prev = linear_acc_curr;

        imu_data_buff_.pop_front();
    }
    
    return true;
}

bool Activity::PublishPose() {
    // a. set header:
    message_odom_.header.stamp = ros::Time::now();
    message_odom_.header.frame_id = odom_config_.frame_id;
    
    // b. set child frame id:
    message_odom_.child_frame_id = odom_config_.frame_id;

    // b. set orientation:
    Eigen::Quaterniond q(pose_.block<3, 3>(0, 0));
    message_odom_.pose.pose.orientation.x = q.x();
    message_odom_.pose.pose.orientation.y = q.y();
    message_odom_.pose.pose.orientation.z = q.z();
    message_odom_.pose.pose.orientation.w = q.w();

    // c. set position:
    Eigen::Vector3d t = pose_.block<3, 1>(0, 3);
    message_odom_.pose.pose.position.x = t.x();
    message_odom_.pose.pose.position.y = t.y();
    message_odom_.pose.pose.position.z = t.z();  

    // d. set velocity:
    message_odom_.twist.twist.linear.x = vel_.x();
    message_odom_.twist.twist.linear.y = vel_.y();
    message_odom_.twist.twist.linear.z = vel_.z(); 

    odom_estimation_pub_.publish(message_odom_);

    return true;
}

} // namespace estimator

} // namespace imu_integration