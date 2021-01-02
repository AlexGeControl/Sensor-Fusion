/*
 * @Description: ceres parameter block for LIO extended pose
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 */

#ifndef LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_PARAM_PRVAG_HPP_
#define LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_PARAM_PRVAG_HPP_

#include <ceres/ceres.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <sophus/so3.hpp>

namespace sliding_window {

class ParamPRVAG : public ceres::LocalParameterization {

  virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
  virtual bool ComputeJacobian(const double *x, double *jacobian) const;
  virtual int GlobalSize() const { return 7; };
  virtual int LocalSize() const { return 6; };

};

} // namespace sliding_window

#endif //LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_PARAM_PRVAG_HPP_