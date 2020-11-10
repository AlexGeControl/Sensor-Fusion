/*
 * @Description: IMU integration activity
 * @Author: Ge Yao
 * @Date: 2020-11-10 14:25:03
 */
#include "imu_integration/activity.hpp"
#include "glog/logging.h"

namespace imu_integration {

Activity::Activity(ros::NodeHandle& nh, std::string cloud_topic) {
    // subscriber:
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/sim/sensor/imu", 1000000);
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

    return true;
}

bool Activity::HasData(void) {
    if (static_cast<size_t>(0) == imu_data_buff_.size())
        return false;

    return true;
}

bool Activity::UpdatePose() {
    return true;
}

bool Activity::PublishPose() {
    return true;
}

}