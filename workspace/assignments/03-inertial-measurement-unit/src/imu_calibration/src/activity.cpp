#include "activity.h"

#include "imu_calibration/AllanVarianceAnalysisResult.h"


namespace imu_calibration {

Activity::Activity()
    : private_nh_("~"),
    as_(nh_, kCalibrateIMUAction, false),
    state_("GNSS_INS_SIM_IMU", 10000) {
}

Activity::~Activity() {}

void Activity::Init(void) {
    // load parameters:
    private_nh_.param("debug_mode", config_.debug_mode, false);
    private_nh_.param("max_playback_rate", config_.max_playback_rate, 120);

    private_nh_.param("imu/device_name", config_.device_name, std::string("GNSS_INS_SIM_IMU"));
    private_nh_.param("imu/topic_name", config_.topic_name, std::string("/sim/sensor/imu"));
    private_nh_.param("imu/allan_variance_curve/output_path", config_.output_path, std::string("."));
    private_nh_.param("imu/allan_variance_curve/min_collection_time_in_mins", config_.min_collection_time_in_mins, 20);
    private_nh_.param("imu/allan_variance_curve/max_num_clusters", config_.max_num_clusters, 2000);

    if (config_.debug_mode) {
        ROS_WARN(
            "[IMU Calibration] params: debug=%s %s %s %s %d %d",
            config_.debug_mode ? "true" : "false",
            config_.topic_name.c_str(),
            config_.device_name.c_str(),
            config_.output_path.c_str(),
            config_.min_collection_time_in_mins,
            config_.max_num_clusters
        );
    }

    // initialize state:
    state_.estimator.SetName(config_.device_name);
    state_.estimator.SetMaxNumClusters(config_.max_num_clusters);

    // initialize action server:
    as_.registerGoalCallback(boost::bind(&Activity::SetMinCollectionTimeCB, this));;
    as_.registerPreemptCallback(boost::bind(&Activity::PreemptCB, this));
    as_.start();

    // initialize services:
    AdvertiseService(kGetIMUCalibrationResults, &Activity::GetAllanVarianceAnalysisResult, this);
}

void Activity::CheckGoal(void) {
    if (as_.isActive()) {
        // get latest timestamp of measurement:
        double time = state_.estimator.GetT();

        // update feedback:
        state_.feedback.collection_time_in_mins = (int)(
            (time - state_.timestamp_start) / 60.0
        );

        // check goal:
        if (state_.feedback.collection_time_in_mins >= config_.min_collection_time_in_mins) {
            if (state_.thread == nullptr) {
                state_.thread = new boost::thread(
                    boost::bind(&Activity::DoEstimate, this)
                );
            } 

            if (
                state_.thread->try_join_for(boost::chrono::milliseconds(5))
            ) {
                // set result, accelerometer:
                state_.estimator.GetAllanVarianceAnalysisResultMsg(state_.result.result);

                // set the action state to succeeded
                as_.setSucceeded(state_.result);
                
                // shutdown subscribers:
                ShutdownSubscribers();

                // remove thread instance:
                delete state_.thread;
                state_.thread = nullptr;

                // indicate ready for get results call:
                state_.feedback.is_result_available = true;
            } else {
                state_.feedback.allan_variance_building_progress = state_.estimator.GetAllanCurveBuildingProgress();
            }
        }

        // publish feedback:
        as_.publishFeedback(state_.feedback);
    }
}

void Activity::SetMinCollectionTimeCB(void) {
    // reset state:
    state_.timestamp_start = -1.0;

    state_.estimator.Reset();
    state_.estimator.SetDebugMode(config_.debug_mode);
    state_.estimator.SetMaxNumClusters(config_.max_num_clusters);

    state_.thread == nullptr;

    state_.feedback.collection_time_in_mins = 0;
    state_.feedback.allan_variance_building_progress = 0.0;
    state_.feedback.is_result_available = false;

    state_.result.result.device_name = "";

    // accept new goal:
    auto goal = as_.acceptNewGoal();

    // load calibration config"
    config_.min_collection_time_in_mins = goal->min_collection_time_in_mins;
    config_.max_num_clusters = goal->max_num_clusters;

    // launch subscribers:
    LaunchSubscribers(config_.max_playback_rate);

    if (config_.debug_mode) {
        ROS_WARN("[IMU Calibration]: New goal accepted.");
    }
}

void Activity::PreemptCB(void) {
    if (as_.isActive()) {
        // set as preempted
        as_.setPreempted();

        // shutdown subscribers:
        ShutdownSubscribers();
    }
}

void Activity::ImuCB(const sensor_msgs::ImuConstPtr& msg) {
    if (
        as_.isActive() && 
        (state_.feedback.collection_time_in_mins < config_.min_collection_time_in_mins)
    ) {
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
}

void Activity::DoEstimate(void) {
    // estimate:
    state_.estimator.Estimate();
}

bool Activity::GetAllanVarianceAnalysisResult(    
    imu_calibration::GetAllanVarianceAnalysisResult::Request& req,
    imu_calibration::GetAllanVarianceAnalysisResult::Response& res
) {
    // clear results:
    res.result.device_name = "";

    if (state_.result.result.device_name == "") {
        // no result available:
        res.status = 1;
    } else {
        state_.estimator.GetAllanVarianceAnalysisResultMsg(res.result);

        // success:
        res.status = 0;
    }

    return true;
}

void Activity::LaunchSubscribers(int queue_size=120) {
    // raw IMU messages:
    Subscribe(config_.topic_name, &Activity::ImuCB, this, queue_size);

    if (config_.debug_mode) {
        ROS_INFO("[IMU Calibration]: Launch subscribers.");
    }
}

void Activity::ShutdownSubscribers(void) {
    // raw IMU messages:
    ShutdownSubscriber(config_.topic_name);

    if (config_.debug_mode) {
        ROS_INFO("[IMU Calibration]: Shutdown subscribers.");
    }
}

}  // namespace imu_calibration