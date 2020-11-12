/*
 * @Description: IMU-lidar fusion for localization workflow
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */
#ifndef LIDAR_LOCALIZATION_FILTERING_FILTERING_HPP_
#define LIDAR_LOCALIZATION_FILTERING_FILTERING_HPP_

#include <deque>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "lidar_localization/sensor_data/cloud_data.hpp"
#include "lidar_localization/sensor_data/pose_data.hpp"

#include "lidar_localization/models/cloud_filter/cloud_filter_interface.hpp"
#include "lidar_localization/models/cloud_filter/box_filter.hpp"

#include "lidar_localization/models/scan_context_manager/scan_context_manager.hpp"

#include "lidar_localization/models/registration/registration_interface.hpp"

#include "lidar_localization/models/kalman_filter/kalman_filter.hpp"

namespace lidar_localization {

class Filtering {
  public:
    Filtering();

    bool Update(
      const CloudData& cloud_data, 
      Eigen::Matrix4f& cloud_pose
    );

    // setters:
    bool SetScanContextPose(const CloudData& init_scan);
    bool SetGNSSPose(const Eigen::Matrix4f& init_pose);

    // getters:
    bool HasInited();
    bool HasNewGlobalMap();
    bool HasNewLocalMap();

    void GetGlobalMap(CloudData::CLOUD_PTR& global_map);
    CloudData::CLOUD_PTR& GetLocalMap();
    CloudData::CLOUD_PTR& GetCurrentScan();

    Eigen::Matrix4f GetInitPose(void);

  private:
    bool InitWithConfig(void);

    // a. filter initializer:
    bool InitFilter(
      std::string filter_user, 
      std::shared_ptr<CloudFilterInterface>& filter_ptr, 
      const YAML::Node& config_node
    );
    bool InitLocalMapSegmenter(const YAML::Node& config_node);
    bool InitFilters(const YAML::Node& config_node);
    // b. map initializer:
    bool InitGlobalMap(const YAML::Node& config_node);
    // c. scan context manager initializer:
    bool InitScanContextManager(const YAML::Node& config_node);
    // d. frontend initializer:
    bool InitRegistration(
      std::shared_ptr<RegistrationInterface>& registration_ptr, 
      const YAML::Node& config_node
    );
    // e. IMU-lidar fusion initializer:
    bool InitFusion(const YAML::Node& config_node);

    bool SetInitPose(const Eigen::Matrix4f& init_pose);
    bool ResetLocalMap(float x, float y, float z);

  private:
    std::string map_path_ = "";
    std::string scan_context_path_ = "";

    std::string loop_closure_method_ = "";

    // a. global map:
    std::shared_ptr<CloudFilterInterface> global_map_filter_ptr_;
    // b. local map:
    std::shared_ptr<BoxFilter> local_map_segmenter_ptr_;
    std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
    // c. current scan:
    std::shared_ptr<CloudFilterInterface> current_scan_filter_ptr_;

    // scan context manager:
    std::shared_ptr<ScanContextManager> scan_context_manager_ptr_;
    // frontend:
    std::shared_ptr<RegistrationInterface> registration_ptr_; 
    // IMU-lidar Kalman filter:
    std::shared_ptr<KalmanFilter> kalman_filter_ptr_;
    
    CloudData::CLOUD_PTR global_map_ptr_;
    CloudData::CLOUD_PTR local_map_ptr_;
    CloudData::CLOUD_PTR current_scan_ptr_;

    Eigen::Matrix4f current_pose_ = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f current_gnss_pose_ = Eigen::Matrix4f::Identity();

    bool has_inited_ = false;
    bool has_new_global_map_ = false;
    bool has_new_local_map_ = false;
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_FILTERING_FILTERING_HPP_