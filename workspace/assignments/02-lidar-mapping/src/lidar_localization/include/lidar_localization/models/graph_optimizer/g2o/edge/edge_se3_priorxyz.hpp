/*
 * @Description: GNSS 坐标做观测时使用的先验边
 * @Author: Ren Qian
 * @Date: 2020-03-01 18:05:35
 */
#ifndef LIDAR_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_G2O_EDGE_EDGE_SE3_PRIORXYZ_HPP_
#define LIDAR_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_G2O_EDGE_EDGE_SE3_PRIORXYZ_HPP_

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

namespace g2o {
class EdgeSE3PriorXYZ : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3> {
  public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	EdgeSE3PriorXYZ()
    	:g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3>() {
	}

	void computeError() override {
		const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]);

		Eigen::Vector3d estimate = v1->estimate().translation();
		_error = estimate - _measurement;
	}

    void setMeasurement(const Eigen::Vector3d& m) override {
		_measurement = m;
	}

	virtual bool read(std::istream& is) override {
    	Eigen::Vector3d v;
		is >> v(0) >> v(1) >> v(2);

    	setMeasurement(Eigen::Vector3d(v));

		for (int i = 0; i < information().rows(); ++i) {
			for (int j = i; j < information().cols(); ++j) {
				is >> information()(i, j);
				// update cross-diagonal element:
				if (i != j) {
					information()(j, i) = information()(i, j);
				}
			}
		}

		return true;
	}

	virtual bool write(std::ostream& os) const override {
    	Eigen::Vector3d v = _measurement;
		os << v(0) << " " << v(1) << " " << v(2) << " ";
		for (int i = 0; i < information().rows(); ++i)
			for (int j = i; j < information().cols(); ++j)
				os << " " << information()(i, j);
		return os.good();
	}
};
}

#endif
