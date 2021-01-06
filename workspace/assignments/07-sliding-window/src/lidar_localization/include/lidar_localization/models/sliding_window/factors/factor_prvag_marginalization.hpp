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

  FactorPRVAGMarginalization(void) {}

  void SetResMapMatchingPose(
    const ceres::CostFunction *residual,
    const std::vector<double *> &parameter_blocks
  ) {
    res_map_matching_pose_ = ResidualBlockInfo(residual, parameter_blocks);
  }

  void SetResRelativePose(
    const ceres::CostFunction *residual,
    const std::vector<double *> &parameter_blocks
  ) {
    res_relative_pose_ = ResidualBlockInfo(residual, parameter_blocks);
  }

  void SetResIMUPreIntegration(
    const ceres::CostFunction *residual,
    const std::vector<double *> &parameter_blocks
  ) {
    res_imu_pre_integration_ = ResidualBlockInfo(residual, parameter_blocks);
  }

  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {		
    return true;
  }

private:
  struct ResidualBlockInfo {
    const ceres::CostFunction *residual = nullptr;
    std::vector<double *> parameter_blocks;

    ResidualBlockInfo(void) {}

    ResidualBlockInfo(
      const ceres::CostFunction *_residual,
      const std::vector<double *> &_parameter_blocks
    ) : residual(_residual), parameter_blocks(_parameter_blocks) {}
  };

  ResidualBlockInfo res_map_matching_pose_;
  ResidualBlockInfo res_relative_pose_;
  ResidualBlockInfo res_imu_pre_integration_;

  static void Evaluate(
    ResidualBlockInfo &residual_info,
    Eigen::VectorXd &residuals,
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> &jacobians
  ) {
    // init residual output:
    const int D = static_cast<int>(residual_info.residual->num_residuals());
    residuals.resize(D);

    // init jacobians output:
    std::vector<int> block_sizes = residual_info.residual->parameter_block_sizes();
    const int N = static_cast<int>(block_sizes.size());

    double **raw_jacobians = new double *[N];
    jacobians.resize(N);

    // create raw pointer adaptor:
    for (int i = 0; i < N; i++) {
      jacobians[i].resize(D, block_sizes[i]);
      raw_jacobians[i] = jacobians[i].data();
    }

    residual_info.residual->Evaluate(
      residual_info.parameter_blocks.data(), 
      residuals.data(), raw_jacobians
    );
  }
};

} // namespace sliding_window

#endif // LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_MARGINALIZATION_HPP_
