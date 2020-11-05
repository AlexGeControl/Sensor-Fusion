#include "activity.h"

namespace imu {

namespace calibrator {

Activity::Activity()
    : private_nh_("~"),
    state_("VIO_IMU", 10000) {
}

Activity::~Activity() {}

void Activity::Init(void) {
    // load parameters:
    private_nh_.param("debug_mode", config_.debug_mode, false);
    private_nh_.param("max_playback_rate", config_.max_playback_rate, 120);

    private_nh_.param("imu/device_name", config_.device_name, std::string("GNSS_INS_SIM_IMU"));
    private_nh_.param("imu/topic_name", config_.topic_name, std::string("/sim/sensor/imu"));
    private_nh_.param("imu/allan_variance_curve/output_filename", config_.output_filename, std::string("."));
    private_nh_.param("imu/allan_variance_curve/min_collection_time_in_mins", config_.min_collection_time_in_mins, 120);
    private_nh_.param("imu/allan_variance_curve/max_num_clusters", config_.max_num_clusters, 10000);

    if (config_.debug_mode) {
        ROS_WARN(
            "[IMU Calibration] params: debug=%s %s %s %s %d %d",
            config_.debug_mode ? "true" : "false",
            config_.topic_name.c_str(),
            config_.device_name.c_str(),
            config_.output_filename.c_str(),
            config_.min_collection_time_in_mins,
            config_.max_num_clusters
        );
    }

    // initialize state:
    state_.estimator.SetDebugMode(config_.debug_mode);
    state_.estimator.SetName(config_.device_name);
    state_.estimator.SetMaxNumClusters(config_.max_num_clusters);
}

void Activity::AddMeasurement(const sensor_msgs::ImuConstPtr &msg) {
    // timestamp as seconds:
    double time = msg->header.stamp.toSec();
    
    if (state_.timestamp_start < 0.0) {
        state_.timestamp_start = time;
    }

    // add IMU record:
    state_.estimator.Add(
        time, 
        msg->angular_velocity,
        msg->linear_acceleration
    );
}

void Activity::DoEstimate(void) {
    // estimate:
    state_.estimator.Estimate();
}

void Activity::WriteResults(void) {
    // write results as json file:
    state_.estimator.WriteIMUCalibrationResult(config_.output_filename);
}

}  // namespace calibrator

}  // namespace imu