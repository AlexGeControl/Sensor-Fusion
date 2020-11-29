/*
 * @Description: GNSS/IMU measurement preprocess for ESKF observability analysis
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */
#include <ros/ros.h>
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/data_pretreat/eskf_preprocess_flow.hpp"

using namespace lidar_localization;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "eskf_preprocess_node");
    ros::NodeHandle nh;

    // subscribe to
    // a. raw GNSS/IMU measurement
    // b. reference trajectory
    // publish
    // a. synced GNSS/IMU for ESKF fusion
    // b. synced reference trajectory for evo evaluation
    std::shared_ptr<ESKFPreprocessFlow> eskf_preprocess_flow_ptr = std::make_shared<ESKFPreprocessFlow>(
        nh
    );

    // pre-process GNSS/IMU point cloud at 100Hz:
    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        eskf_preprocess_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}