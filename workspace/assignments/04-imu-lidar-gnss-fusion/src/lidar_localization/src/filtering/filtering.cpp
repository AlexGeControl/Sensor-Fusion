/*
 * @Description: IMU-lidar fusion for localization workflow
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */
#include "lidar_localization/filtering/filtering.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"

#include "lidar_localization/models/cloud_filter/no_filter.hpp"
#include "lidar_localization/models/cloud_filter/voxel_filter.hpp"

#include "lidar_localization/models/registration/ndt_registration.hpp"


namespace lidar_localization {

Filtering::Filtering() : 
    global_map_ptr_(new CloudData::CLOUD()),
    local_map_ptr_(new CloudData::CLOUD()),
    current_scan_ptr_(new CloudData::CLOUD()) 
{   
    // load ROS config:
    InitWithConfig();
}

bool Filtering::Init(
    const CloudData& init_scan,
    const Eigen::Vector3f &init_vel,
    const IMUData &init_imu_data
) {
    if ( SetInitScan(init_scan) ) {
        current_vel_ = init_vel;

        kalman_filter_ptr_->Init(
            current_pose_.cast<double>(),
            current_vel_.cast<double>(),
            init_imu_data
        );
        
        return true;
    }

    return false;
}

bool Filtering::Init(
    const Eigen::Matrix4f& init_pose,
    const Eigen::Vector3f &init_vel,
    const IMUData &init_imu_data
) {
    if ( SetInitGNSS(init_pose) ) {
        current_vel_ = init_vel;

        kalman_filter_ptr_->Init(
            current_pose_.cast<double>(),
            current_vel_.cast<double>(),
            init_imu_data
        );
        
        return true;
    }

    return false;
}

bool Filtering::Update(
    const IMUData &imu_data
) {
    if ( kalman_filter_ptr_->Update(imu_data) ) {
        kalman_filter_ptr_->GetOdometry(
            current_pose_, current_vel_
        );
        return true;
    }

    return false;
}

bool Filtering::Correct(
    const IMUData &imu_data,
    const CloudData& cloud_data, 
    Eigen::Matrix4f& cloud_pose
) {
    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
    static Eigen::Matrix4f last_pose = current_pose_;
    static Eigen::Matrix4f predict_pose = current_pose_;

    // remove invalid measurements:
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, *cloud_data.cloud_ptr, indices);

    // downsample:
    CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD());
    current_scan_filter_ptr_->Filter(cloud_data.cloud_ptr, filtered_cloud_ptr);

    if (!has_inited_) {
        predict_pose = current_gnss_pose_;
    }

    // matching:
    CloudData::CLOUD_PTR result_cloud_ptr(new CloudData::CLOUD());
    registration_ptr_->ScanMatch(filtered_cloud_ptr, predict_pose, result_cloud_ptr, cloud_pose);
    pcl::transformPointCloud(*cloud_data.cloud_ptr, *current_scan_ptr_, cloud_pose);

    // update predicted pose:
    step_pose = last_pose.inverse() * cloud_pose;
    predict_pose = cloud_pose * step_pose;
    last_pose = cloud_pose;

    kalman_filter_ptr_->Correct(
        imu_data,
        cloud_data.time, cloud_pose
    );

    // shall the local map be updated:
    std::vector<float> edge = local_map_segmenter_ptr_->GetEdge();
    for (int i = 0; i < 3; i++) {
        if (
            fabs(cloud_pose(i, 3) - edge.at(2 * i)) > 50.0 &&
            fabs(cloud_pose(i, 3) - edge.at(2 * i + 1)) > 50.0
        ) {
            continue;
        }
            
        ResetLocalMap(
            cloud_pose(0,3), 
            cloud_pose(1,3), 
            cloud_pose(2,3)
        );
        break;
    }

    return true;
}

void Filtering::GetGlobalMap(CloudData::CLOUD_PTR& global_map) {
    // downsample global map for visualization:
    global_map_filter_ptr_->Filter(global_map_ptr_, global_map);
    has_new_global_map_ = false;
}

void Filtering::GetOdometry(Eigen::Matrix4f &pose, Eigen::Vector3f &vel) {
    pose = current_pose_;
    vel = current_vel_;
}

bool Filtering::InitWithConfig(void) {
    std::string config_file_path = WORK_SPACE_PATH + "/config/filtering/filtering.yaml";

    YAML::Node config_node = YAML::LoadFile(config_file_path);

    LOG(INFO) << std::endl
              << "-----------------Init IMU-Lidar Fusion for Localization-------------------" 
              << std::endl;
    
    // a. init filters:
    InitFilters(config_node);
    // b. init map:
    InitGlobalMap(config_node);
    // c. init scan context manager:
    InitScanContextManager(config_node);
    // d. init frontend:
    InitRegistration(registration_ptr_, config_node);
    // e. init fusion:
    InitFusion(config_node);

    // init local map for frontend matching:
    ResetLocalMap(0.0, 0.0, 0.0);

    return true;
}

bool Filtering::InitFilter(
    std::string filter_user, 
    std::shared_ptr<CloudFilterInterface>& filter_ptr, 
    const YAML::Node& config_node
) {
    std::string filter_mothod = config_node[filter_user + "_filter"].as<std::string>();

    std::cout << "\tFilter Method for " << filter_user << ": " << filter_mothod << std::endl;

    if (filter_mothod == "voxel_filter") {
        filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_mothod][filter_user]);
    } else if (filter_mothod == "no_filter") {
        filter_ptr = std::make_shared<NoFilter>();
    } else {
        LOG(ERROR) << "Filter method " << filter_mothod << " for " << filter_user << " NOT FOUND!";
        return false;
    }

    return true;
}

bool Filtering::InitLocalMapSegmenter(const YAML::Node& config_node) {
    local_map_segmenter_ptr_ = std::make_shared<BoxFilter>(config_node);
    return true;
}

bool Filtering::InitFilters(const YAML::Node& config_node) {
    // a. global map filter -- downsample point cloud map for visualization:
    InitFilter("global_map", global_map_filter_ptr_, config_node);
    // b. local map filter -- downsample & ROI filtering for scan-map matching:
    InitLocalMapSegmenter(config_node);
    InitFilter("local_map", local_map_filter_ptr_, config_node);
    // c. scan filter -- 
    InitFilter("current_scan", current_scan_filter_ptr_, config_node);
}

bool Filtering::InitGlobalMap(const YAML::Node& config_node) {
    map_path_ = config_node["map_path"].as<std::string>();

    pcl::io::loadPCDFile(map_path_, *global_map_ptr_);
    LOG(INFO) << "Load global map, size:" << global_map_ptr_->points.size();

    // since scan-map matching is used, here apply the same filter to local map & scan:
    local_map_filter_ptr_->Filter(global_map_ptr_, global_map_ptr_);
    LOG(INFO) << "Filtered global map, size:" << global_map_ptr_->points.size();

    has_new_global_map_ = true;

    return true;
}

bool Filtering::InitScanContextManager(const YAML::Node& config_node) {
    // get loop closure config:
    loop_closure_method_ = config_node["loop_closure_method"].as<std::string>();

    // create instance:
    scan_context_manager_ptr_ = std::make_shared<ScanContextManager>(config_node[loop_closure_method_]);

    // load pre-built index:
    scan_context_path_ = config_node["scan_context_path"].as<std::string>();
    scan_context_manager_ptr_->Load(scan_context_path_);

    return true;
}

bool Filtering::InitRegistration(
    std::shared_ptr<RegistrationInterface>& registration_ptr, 
    const YAML::Node& config_node
) {
    std::string registration_method = config_node["registration_method"].as<std::string>();

    std::cout << "\tPoint Cloud Registration Method: " << registration_method << std::endl;

    if (registration_method == "NDT") {
        registration_ptr = std::make_shared<NDTRegistration>(config_node[registration_method]);
    } else {
        LOG(ERROR) << "Registration method " << registration_method << " NOT FOUND!";
        return false;
    }

    return true;
}

bool Filtering::InitFusion(const YAML::Node& config_node) {
    std::string fusion_method = config_node["fusion_method"].as<std::string>();

    std::cout << "\tIMU-Lidar-GNSS Fusion Method: " << fusion_method << std::endl;

    if (fusion_method == "kalman_filter") {
        kalman_filter_ptr_ = std::make_shared<KalmanFilter>(config_node[fusion_method]);
    } else {
        LOG(ERROR) << "Fusion method " << fusion_method << " NOT FOUND!";
        return false;
    }

    return true;
}

/**
 * @brief  get init pose using scan context matching
 * @param  init_scan, init key scan
 * @return true if success otherwise false
 */
bool Filtering::SetInitScan(const CloudData& init_scan) {
    // get init pose proposal using scan context match:
    Eigen::Matrix4f init_pose =  Eigen::Matrix4f::Identity();
    if (
        !scan_context_manager_ptr_->DetectLoopClosure(init_scan, init_pose)
    ) {
        return false;
    }

    // set init pose:
    SetInitPose(init_pose);
    has_inited_ = true;
    
    return true;
}

bool Filtering::SetInitGNSS(const Eigen::Matrix4f& gnss_pose) {
    static int gnss_cnt = 0;

    current_gnss_pose_ = gnss_pose;

    if (gnss_cnt == 0) {
        SetInitPose(gnss_pose);
    } else if (gnss_cnt > 3) {
        has_inited_ = true;
    }
    gnss_cnt++;

    return true;
}

bool Filtering::SetInitPose(const Eigen::Matrix4f& init_pose) {
    current_pose_ = init_pose;

    ResetLocalMap(
        init_pose(0,3), 
        init_pose(1,3), 
        init_pose(2,3)
    );

    return true;
}

bool Filtering::ResetLocalMap(
    float x, 
    float y, 
    float z
) {
    std::vector<float> origin = {x, y, z};

    // segment local map from global map:
    local_map_segmenter_ptr_->SetOrigin(origin);
    local_map_segmenter_ptr_->Filter(global_map_ptr_, local_map_ptr_);

    registration_ptr_->SetInputTarget(local_map_ptr_);

    has_new_local_map_ = true;

    std::vector<float> edge = local_map_segmenter_ptr_->GetEdge();

    LOG(INFO) << "New local map:" 
              << edge.at(0) << ","
              << edge.at(1) << ","
              << edge.at(2) << ","
              << edge.at(3) << ","
              << edge.at(4) << ","
              << edge.at(5) << std::endl << std::endl;

    return true;
}

} // namespace lidar_localization