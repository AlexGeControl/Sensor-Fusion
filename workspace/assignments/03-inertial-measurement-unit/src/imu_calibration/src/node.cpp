#include "activity.h"
#include <ros/ros.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

int main(int argc, char** argv) {
    std::string node_name{"imu_calibration_node"};
    ros::init(argc, argv, node_name);
    
    imu::calibrator::Activity calibrator_activity;

    calibrator_activity.Init();
    
    const std::string bag_path = "/workspace/data/gnss_ins_sim/allan_variance_analysis/allan_variance_analysis.bag";
    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string("/sim/sensor/imu"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    foreach(rosbag::MessageInstance const m, view)
    {
        sensor_msgs::Imu::ConstPtr i = m.instantiate<sensor_msgs::Imu>();
        if (i != NULL) {
            calibrator_activity.AddMeasurement(i);
        }
    }
    calibrator_activity.DoEstimate();
    calibrator_activity.WriteResults();
    
    bag.close();
    
    return 0;
}