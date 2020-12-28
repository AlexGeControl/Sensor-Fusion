/*
 * @Description: 前端里程计的node文件
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:56:27
 */
#include <ros/ros.h>
#include "glog/logging.h"

#include <lidar_localization/optimizeMap.h>
#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/mapping/back_end/back_end_flow.hpp"

using namespace lidar_localization;

std::shared_ptr<BackEndFlow> _back_end_flow_ptr;
bool _need_optimize_map = false;

bool optimize_map_callback(optimizeMap::Request &request, optimizeMap::Response &response) {
    _need_optimize_map = true;
    response.succeed = true;
    return response.succeed;
}

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "back_end_node");
    ros::NodeHandle nh;

    std::string cloud_topic, odom_topic;
    nh.param<std::string>("cloud_topic", cloud_topic, "/synced_cloud");
    nh.param<std::string>("odom_topic", odom_topic, "/laser_odom");

    //
    // subscribe to:
    // 
    // a. undistorted Velodyne measurement:
    // b. lidar pose in map frame:
    // c. lidar odometry estimation:
    // d. loop close pose:
    // publish:
    // a. lidar odometry in map frame:
    // b. key frame pose and corresponding GNSS/IMU pose
    // c. optimized key frame sequence as trajectory
    _back_end_flow_ptr = std::make_shared<BackEndFlow>(nh, cloud_topic, odom_topic);
    ros::ServiceServer service = nh.advertiseService("optimize_map", optimize_map_callback);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        _back_end_flow_ptr->Run();

        if (_need_optimize_map) {
            _back_end_flow_ptr->ForceOptimize();
            _need_optimize_map = false;
        }

        rate.sleep();
    }

    return 0;
}