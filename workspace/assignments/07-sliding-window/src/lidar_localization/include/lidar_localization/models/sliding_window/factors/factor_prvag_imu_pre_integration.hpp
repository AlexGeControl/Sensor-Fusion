/*
 * @Description: ceres residual block for LIO IMU pre-integration measurement
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 */
#ifndef LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_IMU_PRE_INTEGRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_IMU_PRE_INTEGRATION_HPP_

#include <ceres/ceres.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <sophus/so3.hpp>

#include "glog/logging.h"

namespace sliding_window {

class FactorPRVAGIMUPreIntegration : public ceres::SizedCostFunction<15, 15, 15> {
public:
	static const int INDEX_P = 0;
	static const int INDEX_R = 3;
	static const int INDEX_V = 6;
	static const int INDEX_A = 9;
	static const int INDEX_G = 12;

  FactorPRVAGIMUPreIntegration(void) {};

	void SetT(const double &T) {
		T_ = T;
	}

	void SetGravitiy(const Eigen::Vector3d &g) {
		g_ = g;
	}

  void SetMeasurement(const Eigen::VectorXd &m) {
		m_ = m;
	}

  void SetInformation(const Eigen::MatrixXd &I) {
    I_ = I;
  }

	void SetJacobian(const Eigen::MatrixXd &J) {
		J_ = J;
	}

  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    //
    // parse parameters:
    //
    // a. pose i
    Eigen::Vector3d pos_i(
      parameters[0][INDEX_P + 0], parameters[0][INDEX_P + 1], parameters[0][INDEX_P + 2]
    );
    Sophus::SO3d    ori_i = Sophus::SO3d::exp(Eigen::Vector3d(
      parameters[0][INDEX_R + 0], parameters[0][INDEX_R + 1], parameters[0][INDEX_R + 2]
    ));
		Eigen::Vector3d vel_i(
      parameters[0][INDEX_V + 0], parameters[0][INDEX_V + 1], parameters[0][INDEX_V + 2]
    );
		Eigen::Vector3d b_a_i(
      parameters[0][INDEX_A + 0], parameters[0][INDEX_A + 1], parameters[0][INDEX_A + 2]
    );
		Eigen::Vector3d b_g_i(
      parameters[0][INDEX_G + 0], parameters[0][INDEX_G + 1], parameters[0][INDEX_G + 2]
    );

    // b. pose j
    Eigen::Vector3d pos_j(
      parameters[1][INDEX_P + 0], parameters[1][INDEX_P + 1], parameters[1][INDEX_P + 2]
    );
    Sophus::SO3d    ori_j = Sophus::SO3d::exp(Eigen::Vector3d(
      parameters[1][INDEX_R + 0], parameters[1][INDEX_R + 1], parameters[1][INDEX_R + 2]
    ));
		Eigen::Vector3d vel_j(
      parameters[1][INDEX_V + 0], parameters[1][INDEX_V + 1], parameters[1][INDEX_V + 2]
    );
		Eigen::Vector3d b_a_j(
      parameters[1][INDEX_A + 0], parameters[1][INDEX_A + 1], parameters[1][INDEX_A + 2]
    );
		Eigen::Vector3d b_g_j(
      parameters[1][INDEX_G + 0], parameters[1][INDEX_G + 1], parameters[1][INDEX_G + 2]
    );

    //
    // parse measurement:
    // 
		const Eigen::Vector3d &alpha_ij = m_.block<3, 1>(INDEX_P, 0);
		const Eigen::Vector3d &theta_ij = m_.block<3, 1>(INDEX_R, 0);
		const Eigen::Vector3d  &beta_ij = m_.block<3, 1>(INDEX_V, 0);

    //
    // get square root of information matrix:
    //
    Eigen::Matrix<double, 15, 15> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 15, 15>>(
      I_
    ).matrixL().transpose();

    //
    // compute residual:
    //
    Eigen::Map<Eigen::Matrix<double, 15, 1>> error(residuals);

		error.block<3, 1>(INDEX_P, 0) = ori_i.inverse() * (pos_j - pos_i - (vel_i - 0.50 * g_ * T_) * T_) - alpha_ij;
		error.block<3, 1>(INDEX_R, 0) = (Sophus::SO3d::exp(theta_ij).inverse()*ori_i.inverse()*ori_j).log();
		error.block<3, 1>(INDEX_V, 0) = ori_i.inverse() * (vel_j - vel_i + g_ * T_) - beta_ij;
		error.block<3, 1>(INDEX_A, 0) = b_a_j - b_a_i;
		error.block<3, 1>(INDEX_G, 0) = b_g_j - b_g_i;

    error = sqrt_info * error;
		
    return true;
  }

private:
	double T_ = 0.0;

	Eigen::Vector3d g_ = Eigen::Vector3d::Zero();

  Eigen::VectorXd m_;
  Eigen::MatrixXd I_;

	Eigen::MatrixXd J_;
};

} // namespace sliding_window

#endif // LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_IMU_PRE_INTEGRATION_HPP_
