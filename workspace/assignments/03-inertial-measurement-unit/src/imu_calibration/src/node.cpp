#include "activity.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    std::string node_name{"imu_calibration_node"};
    ros::init(argc, argv, node_name);
    
    imu_calibration::Activity activity;

    activity.Init();
    
    // 10 Hz:
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        activity.CheckGoal();
        loop_rate.sleep();
    } 
    
    return 0;
}