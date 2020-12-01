/*
 * @Description: LIO mapping backend, implementation
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 */
#include "lidar_localization/mapping/back_end/lio_back_end.hpp"

#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/tools/file_manager.hpp"

namespace lidar_localization {

LIOBackEnd::LIOBackEnd() {
    InitWithConfig();
}

bool LIOBackEnd::InitWithConfig() {
    std::string config_file_path = WORK_SPACE_PATH + "/config/mapping/lio_back_end.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    std::cout << "-----------------Init LIO Backend-------------------" << std::endl;

    InitDataPath(config_node);
    InitParam(config_node);
    InitGraphOptimizer(config_node);
    InitIMUPreIntegrator(config_node);

    return true;
}

bool LIOBackEnd::InitDataPath(const YAML::Node& config_node) {
    std::string data_path = config_node["data_path"].as<std::string>();
    if (data_path == "./") {
        data_path = WORK_SPACE_PATH;
    }

    if (!FileManager::CreateDirectory(data_path + "/slam_data"))
        return false;

    key_frames_path_ = data_path + "/slam_data/key_frames";
    scan_context_path_ = data_path + "/slam_data/scan_context";
    trajectory_path_ = data_path + "/slam_data/trajectory";

    if (!FileManager::InitDirectory(key_frames_path_, "Point Cloud Key Frames"))
        return false;
    if (!FileManager::InitDirectory(scan_context_path_, "Scan Context Index & Data"))
        return false;
    if (!FileManager::InitDirectory(trajectory_path_, "Estimated Trajectory"))
        return false;

    if (!FileManager::CreateFile(ground_truth_ofs_, trajectory_path_ + "/ground_truth.txt"))
        return false;
    if (!FileManager::CreateFile(laser_odom_ofs_, trajectory_path_ + "/laser_odom.txt"))
        return false;

    return true;
}


bool LIOBackEnd::InitParam(const YAML::Node& config_node) {
    key_frame_distance_ = config_node["key_frame_distance"].as<float>();

    return true;
}

bool LIOBackEnd::InitGraphOptimizer(const YAML::Node& config_node) {
    std::string graph_optimizer_type = config_node["graph_optimizer_type"].as<std::string>();
    if (graph_optimizer_type == "g2o") {
        graph_optimizer_ptr_ = std::make_shared<G2oGraphOptimizer>("lm_var");
    } else {
        LOG(ERROR) << "Optimizer " << graph_optimizer_type << " NOT FOUND!";
        return false;
    }
    std::cout << "\tOptimizer:" << graph_optimizer_type << std::endl << std::endl;

    graph_optimizer_config_.use_gnss = config_node["use_gnss"].as<bool>();
    graph_optimizer_config_.use_loop_close = config_node["use_loop_close"].as<bool>();
    graph_optimizer_config_.use_imu_pre_integration = config_node["use_imu_pre_integration"].as<bool>();

    graph_optimizer_config_.optimize_step_with_key_frame = config_node["optimize_step_with_key_frame"].as<int>();
    graph_optimizer_config_.optimize_step_with_gnss = config_node["optimize_step_with_gnss"].as<int>();
    graph_optimizer_config_.optimize_step_with_loop = config_node["optimize_step_with_loop"].as<int>();

    // x-y-z & yaw-roll-pitch
    for (int i = 0; i < 6; ++i) {
        graph_optimizer_config_.odom_edge_noise(i) =
            config_node[graph_optimizer_type + "_param"]["odom_edge_noise"][i].as<double>();
        graph_optimizer_config_.close_loop_noise(i) =
            config_node[graph_optimizer_type + "_param"]["close_loop_noise"][i].as<double>();
    }

    // x-y-z:
    for (int i = 0; i < 3; i++) {
        graph_optimizer_config_.gnss_noise(i) =
            config_node[graph_optimizer_type + "_param"]["gnss_noise"][i].as<double>();
    }

    return true;
}

bool LIOBackEnd::InitIMUPreIntegrator(const YAML::Node& config_node) {
    imu_pre_integrator_ptr_ = nullptr;
    
    if (graph_optimizer_config_.use_imu_pre_integration) {
        imu_pre_integrator_ptr_ = std::make_shared<IMUPreIntegrator>(config_node["imu_pre_integration"]);
    }

    return true;
}

bool LIOBackEnd::InsertLoopPose(const LoopPose& loop_pose) {
    if (!graph_optimizer_config_.use_loop_close)
        return false;

    Eigen::Isometry3d isometry;
    isometry.matrix() = loop_pose.pose.cast<double>();
    graph_optimizer_ptr_->AddSe3Edge(
        loop_pose.index0, loop_pose.index1, 
        isometry, 
        graph_optimizer_config_.close_loop_noise
    );

    new_loop_cnt_ ++;
    
    LOG(INFO) << "Add loop closure: " << loop_pose.index0 << "," << loop_pose.index1 << std::endl;

    return true;
}

bool LIOBackEnd::UpdateIMUPreIntegration(const IMUData &imu_data) {
    if ( !graph_optimizer_config_.use_imu_pre_integration || nullptr == imu_pre_integrator_ptr_ )
        return false;
    
    if (
        !imu_pre_integrator_ptr_->IsInited() ||
        imu_pre_integrator_ptr_->Update(imu_data)
    ) {
        return true;
    }

    return false;
}

bool LIOBackEnd::Update(
    const CloudData& cloud_data, 
    const PoseData& laser_odom, 
    const PoseData& gnss_pose,
    const IMUData &imu_data
) {
    ResetParam();

    if ( MaybeNewKeyFrame(cloud_data, laser_odom, gnss_pose, imu_data) ) {
        // write GNSS/IMU pose and lidar odometry estimation as trajectory for evo evaluation:
        SavePose(ground_truth_ofs_, gnss_pose.pose);
        SavePose(laser_odom_ofs_, laser_odom.pose);
        AddNodeAndEdge(gnss_pose);
        
        if (MaybeOptimized()) {
            SaveOptimizedPose();
        }
    }

    return true;
}

void LIOBackEnd::ResetParam() {
    has_new_key_frame_ = false;
    has_new_optimized_ = false;
}

bool LIOBackEnd::MaybeNewKeyFrame(
    const CloudData& cloud_data, 
    const PoseData& laser_odom, 
    const PoseData& gnss_odom,
    const IMUData &imu_data
) {
    static int count = 0;
    static PoseData last_laser_pose = laser_odom;
    static PoseData last_gnss_pose = gnss_odom;

    if (key_frames_deque_.size() == 0) {
        if ( imu_pre_integrator_ptr_ ) {
            imu_pre_integrator_ptr_->Init(imu_data);
        }

        last_laser_pose = laser_odom;
        last_gnss_pose = gnss_odom;

        has_new_key_frame_ = true;
    }

    // whether the current scan is far away enough from last key frame:
    if (fabs(laser_odom.pose(0,3) - last_laser_pose.pose(0,3)) + 
        fabs(laser_odom.pose(1,3) - last_laser_pose.pose(1,3)) +
        fabs(laser_odom.pose(2,3) - last_laser_pose.pose(2,3)) > key_frame_distance_) {

        if ( imu_pre_integrator_ptr_ ) {
            imu_pre_integrator_ptr_->Reset(imu_data, imu_pre_integration_);

            //
            // for IMU pre-integration debugging ONLY:
            // this is critical to IMU pre-integration verification
            //
            if ( 0 == (++count) % 10 ) {
                ShowIMUPreIntegrationResidual(last_gnss_pose, gnss_odom, imu_pre_integration_); 
            } 
            
            last_laser_pose = laser_odom;
            last_gnss_pose = gnss_odom;
        
            has_new_key_frame_ = true;
        }
    }

    // if so:
    if (has_new_key_frame_) {
        // a. first write new key scan to disk:
        std::string file_path = key_frames_path_ + "/key_frame_" + std::to_string(key_frames_deque_.size()) + ".pcd";
        pcl::io::savePCDFileBinary(file_path, *cloud_data.cloud_ptr);
        current_key_scan_.time = cloud_data.time;
        current_key_scan_.cloud_ptr.reset(
            new CloudData::CLOUD(*cloud_data.cloud_ptr)
        );

        // b. create key frame index for lidar scan:
        KeyFrame key_frame;
        key_frame.time = laser_odom.time;
        key_frame.index = (unsigned int)key_frames_deque_.size();
        key_frame.pose = laser_odom.pose;
        key_frames_deque_.push_back(key_frame);
        current_key_frame_ = key_frame;

        // c. create key frame index for GNSS/IMU pose:
        current_key_gnss_.time = gnss_odom.time;
        current_key_gnss_.index = key_frame.index;
        current_key_gnss_.pose = gnss_odom.pose;
    }

    return has_new_key_frame_;
}

bool LIOBackEnd::AddNodeAndEdge(const PoseData& gnss_data) {
    static KeyFrame last_key_frame = current_key_frame_;

    // add node for new key frame pose:
    Eigen::Isometry3d isometry;
    isometry.matrix() = current_key_frame_.pose.cast<double>();
    // fix the pose of the first key frame:
    if (!graph_optimizer_config_.use_gnss && graph_optimizer_ptr_->GetNodeNum() == 0)
        graph_optimizer_ptr_->AddSe3Node(isometry, true);
    else
        graph_optimizer_ptr_->AddSe3Node(isometry, false);
    new_key_frame_cnt_ ++;

    // add edge for new key frame:
    int node_num = graph_optimizer_ptr_->GetNodeNum();
    if (node_num > 1) {
        Eigen::Matrix4f relative_pose = last_key_frame.pose.inverse() * current_key_frame_.pose;
        isometry.matrix() = relative_pose.cast<double>();
        graph_optimizer_ptr_->AddSe3Edge(node_num-2, node_num-1, isometry, graph_optimizer_config_.odom_edge_noise);
    }
    last_key_frame = current_key_frame_;

    // add prior for new key frame pose using GNSS position:
    if (graph_optimizer_config_.use_gnss) {
        Eigen::Vector3d xyz(
            static_cast<double>(gnss_data.pose(0,3)),
            static_cast<double>(gnss_data.pose(1,3)),
            static_cast<double>(gnss_data.pose(2,3))
        );
        graph_optimizer_ptr_->AddSe3PriorXYZEdge(node_num - 1, xyz, graph_optimizer_config_.gnss_noise);
        new_gnss_cnt_ ++;
    }

    return true;
}

bool LIOBackEnd::MaybeOptimized() {
    bool need_optimize = false; 

    if (
        new_key_frame_cnt_ >= graph_optimizer_config_.optimize_step_with_key_frame ||
        new_gnss_cnt_ >= graph_optimizer_config_.optimize_step_with_gnss ||
        new_loop_cnt_ >= graph_optimizer_config_.optimize_step_with_loop
    ) {
        need_optimize = true;
    }
    
    if (!need_optimize)
        return false;

    // reset key frame counters:
    new_key_frame_cnt_ = new_gnss_cnt_ = new_loop_cnt_ = 0;

    if (graph_optimizer_ptr_->Optimize())
        has_new_optimized_ = true;

    return true;
}

bool LIOBackEnd::SavePose(std::ofstream& ofs, const Eigen::Matrix4f& pose) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            ofs << pose(i, j);
            
            if (i == 2 && j == 3) {
                ofs << std::endl;
            } else {
                ofs << " ";
            }
        }
    }

    return true;
}

bool LIOBackEnd::SaveOptimizedPose() {
    if (graph_optimizer_ptr_->GetNodeNum() == 0)
        return false;

    if (!FileManager::CreateFile(optimized_pose_ofs_, trajectory_path_ + "/optimized.txt"))
        return false;

    graph_optimizer_ptr_->GetOptimizedPose(optimized_pose_);

    for (size_t i = 0; i < optimized_pose_.size(); ++i) {
        SavePose(optimized_pose_ofs_, optimized_pose_.at(i));
    }

    return true;
}

bool LIOBackEnd::ForceOptimize() {
    if (graph_optimizer_ptr_->Optimize())
        has_new_optimized_ = true;

    SaveOptimizedPose();

    return has_new_optimized_;
}

void LIOBackEnd::GetOptimizedKeyFrames(std::deque<KeyFrame>& key_frames_deque) {
    KeyFrame key_frame;
    for (size_t i = 0; i < optimized_pose_.size(); ++i) {
        key_frame.pose = optimized_pose_.at(i);
        key_frame.index = (unsigned int)i;
        key_frames_deque.push_back(key_frame);
    }
}

bool LIOBackEnd::HasNewKeyFrame() {
    return has_new_key_frame_;
}

bool LIOBackEnd::HasNewOptimized() {
    return has_new_optimized_;
}

void LIOBackEnd::GetLatestKeyScan(CloudData& key_scan) {
    key_scan.time = current_key_scan_.time;
    key_scan.cloud_ptr.reset(
        new CloudData::CLOUD(*current_key_scan_.cloud_ptr)
    );
}

void LIOBackEnd::GetLatestKeyFrame(KeyFrame& key_frame) {
    key_frame = current_key_frame_;
}

void LIOBackEnd::GetLatestKeyGNSS(KeyFrame& key_frame) {
    key_frame = current_key_gnss_;
}

void LIOBackEnd::ShowIMUPreIntegrationResidual(
    const PoseData &last_gnss_pose, const PoseData& curr_gnss_pose,
    const IMUPreIntegrator::IMUPreIntegration &imu_pre_integration
) {
    const double &T = imu_pre_integration.T_;
    const Eigen::Vector3d &g = imu_pre_integration.g_;

    Eigen::Vector3d r_p = last_gnss_pose.pose.block<3, 3>(0, 0).transpose().cast<double>() * (
        curr_gnss_pose.pose.block<3, 1>(0, 3).cast<double>() - last_gnss_pose.pose.block<3, 1>(0, 3).cast<double>() - 
        ( last_gnss_pose.pose.block<3, 3>(0, 0).cast<double>()*last_gnss_pose.vel.cast<double>() - 0.50 * g * T ) * T
    );

    Sophus::SO3d prev_theta(Eigen::Quaterniond(last_gnss_pose.pose.block<3, 3>(0, 0).cast<double>()));
    Sophus::SO3d curr_theta(Eigen::Quaterniond(curr_gnss_pose.pose.block<3, 3>(0, 0).cast<double>()));
    Sophus::SO3d r_q = prev_theta.inverse() * curr_theta;

    Eigen::Vector3d r_v = last_gnss_pose.pose.block<3, 3>(0, 0).transpose().cast<double>() * (
        curr_gnss_pose.pose.block<3, 3>(0, 0).cast<double>()*curr_gnss_pose.vel.cast<double>() - 
        last_gnss_pose.pose.block<3, 3>(0, 0).cast<double>()*last_gnss_pose.vel.cast<double>() + 
        g * T
    );

    LOG(INFO) << "IMU Pre-Integration Measurement: " << std::endl
                << "\tT: " << T << " --- " << curr_gnss_pose.time - last_gnss_pose.time << std::endl
                << "\talpha:" << std::endl
                << "\t\t" << imu_pre_integration.alpha_ij_.x() << ", " << imu_pre_integration.alpha_ij_.y() << ", " << imu_pre_integration.alpha_ij_.z() << std::endl
                << "\t\t" << r_p.x() << ", " << r_p.y() << ", " << r_p.z() << std::endl
                << "\ttheta:" << std::endl
                << "\t\t" << imu_pre_integration.theta_ij_.angleX() << ", " << imu_pre_integration.theta_ij_.angleY() << ", " << imu_pre_integration.theta_ij_.angleZ() << std::endl
                << "\t\t" << r_q.angleX() << ", " << r_q.angleY() << ", " << r_q.angleZ() << std::endl
                << "\tbeta:" << std::endl
                << "\t\t" << imu_pre_integration.beta_ij_.x() << ", " << imu_pre_integration.beta_ij_.y() << ", " << imu_pre_integration.beta_ij_.z() << std::endl
                << "\t\t" << r_v.x() << ", " << r_v.y() << ", " << r_v.z() << std::endl
                << "\tbias_accel:" 
                << imu_pre_integration.b_a_i_.x() << ", "
                << imu_pre_integration.b_a_i_.y() << ", " 
                << imu_pre_integration.b_a_i_.z()
                << std::endl
                << "\tbias_gyro:" 
                << imu_pre_integration.b_g_i_.x() << ", "
                << imu_pre_integration.b_g_i_.y() << ", " 
                << imu_pre_integration.b_g_i_.z()
                << std::endl
                << "\tcovariance:" << std::endl
                << "\t\talpha: "
                << imu_pre_integration.P_( 0,  0) << ", "
                << imu_pre_integration.P_( 1,  1) << ", " 
                << imu_pre_integration.P_( 2,  3)
                << std::endl
                << "\t\ttheta: "
                << imu_pre_integration.P_( 3,  3) << ", "
                << imu_pre_integration.P_( 4,  4) << ", " 
                << imu_pre_integration.P_( 5,  5)
                << std::endl
                << "\t\tbeta: "
                << imu_pre_integration.P_( 6,  6) << ", "
                << imu_pre_integration.P_( 7,  7) << ", " 
                << imu_pre_integration.P_( 8,  8)
                << std::endl
                << "\t\tbias_accel: "
                << imu_pre_integration.P_( 9,  9) << ", "
                << imu_pre_integration.P_(10, 10) << ", " 
                << imu_pre_integration.P_(11, 11)
                << std::endl
                << "\t\tbias_gyro: "
                << imu_pre_integration.P_(12, 12) << ", "
                << imu_pre_integration.P_(13, 13) << ", " 
                << imu_pre_integration.P_(14, 14)
                << std::endl
                << "\tJacobian:" << std::endl
                << "\t\td_alpha_d_b_a: " << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 0,  9) << ", " << imu_pre_integration.J_( 0, 10) << ", " << imu_pre_integration.J_( 0, 11) << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 1,  9) << ", " << imu_pre_integration.J_( 1, 10) << ", " << imu_pre_integration.J_( 1, 11) << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 2,  9) << ", " << imu_pre_integration.J_( 2, 10) << ", " << imu_pre_integration.J_( 2, 11) << std::endl
                << "\t\td_alpha_d_b_g: " << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 0, 12) << ", " << imu_pre_integration.J_( 0, 13) << ", " << imu_pre_integration.J_( 0, 14) << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 1, 12) << ", " << imu_pre_integration.J_( 1, 13) << ", " << imu_pre_integration.J_( 1, 14) << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 2, 12) << ", " << imu_pre_integration.J_( 2, 13) << ", " << imu_pre_integration.J_( 2, 14) << std::endl
                << "\t\td_theta_d_b_g: " << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 3, 12) << ", " << imu_pre_integration.J_( 3, 13) << ", " << imu_pre_integration.J_( 3, 14) << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 4, 12) << ", " << imu_pre_integration.J_( 4, 13) << ", " << imu_pre_integration.J_( 4, 14) << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 5, 12) << ", " << imu_pre_integration.J_( 5, 13) << ", " << imu_pre_integration.J_( 5, 14) << std::endl
                << "\t\td_beta_d_b_a: " << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 6,  9) << ", " << imu_pre_integration.J_( 6, 10) << ", " << imu_pre_integration.J_( 6, 11) << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 7,  9) << ", " << imu_pre_integration.J_( 7, 10) << ", " << imu_pre_integration.J_( 7, 11) << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 8,  9) << ", " << imu_pre_integration.J_( 8, 10) << ", " << imu_pre_integration.J_( 8, 11) << std::endl
                << "\t\td_beta_d_b_a: " << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 6, 12) << ", " << imu_pre_integration.J_( 6, 13) << ", " << imu_pre_integration.J_( 6, 14) << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 7, 12) << ", " << imu_pre_integration.J_( 7, 13) << ", " << imu_pre_integration.J_( 7, 14) << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 8, 12) << ", " << imu_pre_integration.J_( 8, 13) << ", " << imu_pre_integration.J_( 8, 14) << std::endl
                << std::endl;
}

} // namespace lidar_localization