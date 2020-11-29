/*
 * @Description: Lidar measurement preprocess node
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */
#include <ros/ros.h>
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/data_pretreat/lidar_preprocess_flow.hpp"

using namespace lidar_localization;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "lidar_preprocess_node");
    ros::NodeHandle nh;

    std::string synced_cloud_topic;
    nh.param<std::string>("cloud_topic", synced_cloud_topic, "/synced_cloud");

    // subscribe to
    // a. raw Velodyne measurement
    // b. raw GNSS/IMU measurement
    // publish
    // a. synced lidar measurement for ESKF fusion
    std::shared_ptr<LidarPreprocessFlow> lidar_preprocess_flow_ptr = std::make_shared<LidarPreprocessFlow>(nh, synced_cloud_topic);

    // pre-process lidar point cloud at 100Hz:
    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        lidar_preprocess_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}