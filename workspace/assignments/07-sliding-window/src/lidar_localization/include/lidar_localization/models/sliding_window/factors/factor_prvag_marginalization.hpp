/*
 * @Description: ceres residual block for sliding window marginalization
 * @Author: Ge Yao
 * @Date: 2021-01-05 21:57:49
 */
#ifndef LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_MARGINALIZATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_MARGINALIZATION_HPP_

#include <ceres/ceres.h>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <sophus/so3.hpp>

#include "glog/logging.h"

namespace sliding_window {

class FactorPRVAGMarginalization : public ceres::SizedCostFunction<15, 15> {
public:
	static const int INDEX_P = 0;
	static const int INDEX_R = 3;
	static const int INDEX_V = 6;
	static const int INDEX_A = 9;
	static const int INDEX_G = 12;

  FactorPRVAGMarginalization(
    const ceres::CostFunction *res_map_matching_pose,
    const ceres::CostFunction *res_relative_pose,
    const ceres::CostFunction *res_imu_pre_integration
  ) {
    Eigen::MatrixXd H_rr = Eigen::MatrixXd::Zero(15, 15);
    Eigen::MatrixXd H_rm = Eigen::MatrixXd::Zero(15, 15);
    Eigen::MatrixXd H_mm = Eigen::MatrixXd::Zero(15, 15);
    Eigen::MatrixXd H_mr = Eigen::MatrixXd::Zero(15, 15);

    Eigen::VectorXd b_r = Eigen::VectorXd::Zero(15);
    Eigen::VectorXd b_m = Eigen::VectorXd::Zero(15);
  };

  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {		
    return true;
  }

private:
  static Eigen::Matrix3d JacobianRInv(const Eigen::Vector3d &w) {
      Eigen::Matrix3d J_r_inv = Eigen::Matrix3d::Identity();

      double theta = w.norm();

      if ( theta > 1e-5 ) {
          Eigen::Vector3d k = w.normalized();
          Eigen::Matrix3d K = Sophus::SO3d::hat(k);
          
          J_r_inv = J_r_inv 
                    + 0.5 * K
                    + (1.0 - (1.0 + std::cos(theta)) * theta / (2.0 * std::sin(theta))) * K * K;
      }

      return J_r_inv;
  }
};

} // namespace sliding_window

#endif // LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_MARGINALIZATION_HPP_
