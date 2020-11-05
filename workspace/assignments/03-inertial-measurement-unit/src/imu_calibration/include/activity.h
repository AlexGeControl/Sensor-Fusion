#ifndef IMU_CALIBRATOR_ACTIVITY_H
#define IMU_CALIBRATOR_ACTIVITY_H

#include <ros/ros.h>

#include <string>
#include <map>

#include <sensor_msgs/Imu.h>
#include <boost/thread/thread.hpp>

#include "node_constants.h"

#include "allan_variance.h"


namespace imu {

namespace calibrator {

struct Config {
    bool debug_mode;
    int max_playback_rate;

    std::string topic_name;
    std::string device_name;
    std::string output_filename;

    int min_collection_time_in_mins;
    int max_num_clusters;
};

struct State {
    double timestamp_start;
    allan_variance::AllanVariance estimator;
    boost::thread* thread;

    State(std::string name, int max_num_clusters) : estimator(false, name, max_num_clusters), thread(nullptr){}
};

class Activity {
public:
    Activity();
    ~Activity();

    void Init(void);
    void AddMeasurement(const sensor_msgs::ImuConstPtr &msg);
    void DoEstimate(void);
    void WriteResults(void); 
private:
    ros::NodeHandle private_nh_;

    Config config_;
    State state_;
};

}  // namespace calibrator

}  // namespace imu

#endif  // IMU_CALIBRATOR_ACTIVITY_H