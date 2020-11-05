
#include <ros/ros.h>

#include <ceres/ceres.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <iostream>
#include <sstream>

#include "CSVWriter.h"
#include "allan_variance.h"

namespace pt = boost::property_tree;

namespace imu {

namespace calibrator {

namespace allan_variance {

AllanVariance::AllanVariance(bool debug_mode, std::string name, int max_num_clusters) {
    config_.debug_mode = debug_mode;
    config_.name = name;
    config_.max_num_clusters = max_num_clusters;

    Reset();
}
AllanVariance::~AllanVariance(void) {}

void AllanVariance::Reset(void) {
    // reset state:
    state_.Reset();
    params_.Reset();
}

void AllanVariance::Add(
    double time, 
    const geometry_msgs::Vector3 &angular_velocity, 
    const geometry_msgs::Vector3 &linear_acceleration 
) {
    Data data = {
        time, 
        {
            angular_velocity.x, angular_velocity.y, angular_velocity.z,
            linear_acceleration.x, linear_acceleration.y, linear_acceleration.z
        }
    };

    // add observation:
    state_.data.push_back(data);
}

void AllanVariance::Estimate(void) {
    // set state:
    SetState();

    // set params:
    SetParams();
}

// get IMU noise message:
void AllanVariance::WriteIMUCalibrationResult(const std::string &output_filename) {
    static std::string FIELD_NAME[NUM_FIELDS] = {
        "gyro_x", "gyro_y", "gyro_z",
        "acc_x", "acc_y", "acc_z",
    };

    // write calibration results as JSON file:
    pt::ptree root;

    // a. summary info:
    root.put("general.device_name", config_.name);
    root.put("general.num_observations", params_.num_observations);
    root.put("general.num_clusters", params_.num_clusters);
    
    // b. measurement properties:
    pt::ptree measurements;
    for (int field = WX; field < NUM_FIELDS; ++field) {
        pt::ptree measurement; 

        measurement.put("Q", params_.Q[field]);
        measurement.put("N", params_.N[field]);
        measurement.put("B", params_.B[field]);
        measurement.put("K", params_.K[field]);
        measurement.put("R", params_.R[field]);       

        measurement.put("measurement_noise", params_.measurement_noise[field]);
        measurement.put("bias_random_walk", params_.random_walk[field]);
        measurement.put("bias_instability", params_.bias_instability[field]);

        measurements.push_back(std::make_pair(FIELD_NAME[field], measurement));
    }
    root.add_child("measurements", measurements);

    pt::json_parser::write_json(output_filename+".json", root);

    // write allan variance curve as CSV file:
    CSVWriter csv(",");
    csv.enableAutoNewRow(7);
    // a. write header:
    csv << "T" 
        << "gyro_x" << "gyro_y" << "gyro_z" 
        << "acc_x" << "acc_y" << "acc_z";
    // b. write contents:
    for (const Point &point: params_.curve_smoothed.point) {
        csv << point.tau 
            << point.covariance[WX] << point.covariance[WY] << point.covariance[WZ] 
            << point.covariance[AX] << point.covariance[AY] << point.covariance[AZ];
    }
    csv.writeToFile(output_filename+".csv");

    if (config_.debug_mode) {
        ROS_ERROR("[IMU Calibration]: result is available at %s", output_filename.data());
    }
}
void AllanVariance::SetStateAveragingFactor(void) {
    if (config_.debug_mode) {
        ROS_ERROR("[IMU Calibration]: SetStateAveragingFactor");
    }

    // calculate averaging factors:
    // a. min averaging factor -- bins should only be created if at least nine measurements can be collected:
    double min_averaging_factor = 9.0;
    // b. max averaging factor:
    double max_averaging_factor = (state_.data.size() >> 1) - min_averaging_factor;
    // c.step size in log space with double precision:
    double step = pow(max_averaging_factor / min_averaging_factor, 1.0 / (config_.max_num_clusters - 1));

    // averaging factors in double:
    std::vector<double> averaging_factor;

    averaging_factor.clear();
    averaging_factor.push_back(min_averaging_factor);
    for (int i = 1; i < config_.max_num_clusters; ++i) {
        averaging_factor.push_back(averaging_factor.at(i - 1) * step);
    }

    // format as int:
    state_.averaging_factor.clear();
    state_.averaging_factor.push_back((int)ceil(min_averaging_factor));
    for (int i = 1; i < config_.max_num_clusters; ++i) {
        int factor = (int)ceil(averaging_factor.at(i));

        if (state_.averaging_factor.back() < factor) {
            state_.averaging_factor.push_back(factor);
        }
    }
}

void AllanVariance::SetStateCurveObserved(void) {
    if (config_.debug_mode) {
        ROS_ERROR(
            "[IMU Calibration]: SetStateCurveObserved Num. Obs: %lu, Num. Clusters: %lu.", 
            (state_.data.size()), (state_.averaging_factor.size())
        );
    }

    // reset:
    state_.curve_observed.point.clear();

    // calculate:
    size_t N = state_.data.size();
    double covariance[NUM_FIELDS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    for (const int &factor: state_.averaging_factor) {
        double tau        = factor * state_.tau;
        double normalizer = 2*(tau*tau)*(N - (factor << 1));
        int upper_bound   = N - (factor << 1);

        for (int field = WX; field < NUM_FIELDS; ++field) {
            covariance[field] = 0.0;
        }

        for (int k = 0; k < upper_bound; k++)
        {
            for (int field = WX; field < NUM_FIELDS; ++field) {
                const std::vector<double> &theta = state_.theta.at(static_cast<Field>(field));

                double deviation = theta.at(k + (factor << 1)) - 2*theta.at(k + factor) + theta.at(k);

                covariance[field] += deviation * deviation;
            }
        }

        for (int field = WX; field < NUM_FIELDS; ++field) {
            covariance[field] /= normalizer;
        }

        // add record:
        Point point = {
            tau, 
            {
                covariance[WX], covariance[WY], covariance[WZ],
                covariance[AX], covariance[AY], covariance[AZ]
            }  
        };
        state_.curve_observed.point.push_back(point);

        // update progress:
        ++state_.num_clusters_built;

        if (config_.debug_mode) {
            ROS_WARN(
                "[IMU Calibration]: SetStateCurveObserved: %d--%f--%f--%d, %f %f %f %f %f %f",
                factor, tau, normalizer, upper_bound,
                covariance[WX], covariance[WY], covariance[WZ], covariance[AX], covariance[AY], covariance[AZ]
            );
        }
    }
}

void AllanVariance::SetState(void) {
    if (config_.debug_mode) {
        ROS_ERROR("[IMU Calibration]: SetState");
    }

    // calculate tau:
    double time_ = state_.data.at(0).time;
    for (const Data &data: state_.data) {
        state_.tau += data.time - time_;
        time_ = data.time;
    }
    state_.tau /= (state_.data.size() - 1);

    // calculate theta using cumulative sum:
    double sum_[NUM_FIELDS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    for (const Data &data: state_.data) {
        for (int field = WX; field < NUM_FIELDS; ++field) {
            std::vector<double> &theta = state_.theta.at(static_cast<Field>(field));

            sum_[field] += data.value[field];

            theta.push_back(sum_[field] * state_.tau);
        }
    }

    // calculate averaging factors:
    SetStateAveragingFactor();

    // set observed allan variance curve:
    SetStateCurveObserved();
}

double AllanVariance::GetCovariance(Field field, const double &tau) {
    // clang-format off
    return  (
        ( params_.Q[field]*params_.Q[field] / (tau*tau) ) + 
        ( params_.N[field]*params_.N[field] / tau ) + 
        ( params_.B[field]*params_.B[field] ) + 
        ( params_.K[field]*params_.K[field] * tau ) + 
        ( params_.R[field]*params_.R[field] * (tau*tau) )
    );
    // clang-format on    
}

Eigen::VectorXd AllanVariance::GetInitialEstimation(Field field) {
    if (config_.debug_mode) {
        ROS_ERROR("[IMU Calibration]: GetInitialEstimation");
    }

    size_t N = state_.curve_observed.point.size();
    int D = 2;

    // init matrix:
    Eigen::MatrixXd A(N, 5);
    Eigen::VectorXd b(N);

    // this is an approximation to allan variance curve:
    for (size_t i = 0; i < N; ++i) {
        // set A:
        for (int d = 0; d < (D << 1) + 1; ++d) {
            A(i, d) = pow((state_.curve_observed.point.at(i).tau), d - D);
        }
        // set b:
        b(i) = (state_.curve_observed.point.at(i).covariance[field]);
    }

    // sovle params:
    Eigen::VectorXd params = A.fullPivHouseholderQr().solve(b);
    
    // take absolute value:
    for (int d = 0; d < (D << 1) + 1; ++d) {
        params(d) = sqrt(abs(params(d)));
    }
    
    return params;
}

Eigen::VectorXd AllanVariance::GetRefinedEstimation(Field field, const Eigen::VectorXd &init_params) {
    if (config_.debug_mode) {
        ROS_ERROR("[IMU Calibration]: GetRefinedEstimation");
    }

    size_t N = state_.curve_observed.point.size();

    // parse init params:
    double params[5] = { init_params[0], init_params[1], init_params[2], init_params[3], init_params[4] };

    // config:
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.logging_type                 = ceres::SILENT;
    options.trust_region_strategy_type   = ceres::DOGLEG;
    options.max_num_iterations           = 10;

    // build optimization problem:
    ceres::Problem problem;
    for (const Point &point: state_.curve_observed.point) {
        ceres::CostFunction* f = new ceres::AutoDiffCostFunction<Error, 1, 5>(
            new Error(
                point.tau, 
                point.covariance[field]
            ) 
        );

        problem.AddResidualBlock(
            f, 
            /* squared loss */
            NULL , 
            params
        );
    }

    // solve:
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary );
    std::cout << summary.FullReport() << std::endl;

    // format result:
    Eigen::VectorXd result(5);
    for (size_t i = 0; i < 5; ++i) {
        result(i) = params[i];
    }

    return result;
}

void AllanVariance::SetParamsCurveSmoothed(void) {
    if (config_.debug_mode) {
        ROS_ERROR("[IMU Calibration]: SetParamsCurveSmoothed");
    }

    // reset:
    params_.curve_smoothed.point.clear();

    // calculate:
    for (const Point &p: state_.curve_observed.point) {
        double tau = p.tau;

        double covariance[NUM_FIELDS];
        for (int field = WX; field < NUM_FIELDS; ++field) {
            covariance[field] = GetCovariance(static_cast<Field>(field), tau);
        }

        // add record:
        Point point = {
            tau, 
            {
                covariance[WX], covariance[WY], covariance[WZ],
                covariance[AX], covariance[AY], covariance[AZ]
            }  
        };
        params_.curve_smoothed.point.push_back(point);
    }   
}

void AllanVariance::SetParamsMeasurementNoise(void) {
    if (config_.debug_mode) {
        ROS_ERROR("[IMU Calibration]: SetParamsMeasurementNoise");
    }

    // calculate measurement noise covariance, ready to be used by ROS message covariance:
    for (int field = WX; field < NUM_FIELDS; ++field) {
        params_.measurement_noise[field] = params_.N[field] * params_.N[field] / sqrt(state_.tau);
    }
}

void AllanVariance::SetParamsRandomWalk(void) {
    if (config_.debug_mode) {
        ROS_ERROR("[IMU Calibration]: SetParamsRandomWalk");
    }

    // calculate bias random walk covariance, ready to be used by ROS message covariance:
    for (int field = WX; field < NUM_FIELDS; ++field) {
        params_.random_walk[field] = params_.K[field] * params_.K[field] * sqrt(state_.tau);
    }
}

void AllanVariance::SetParamsBiasInstability(void) {
    if (config_.debug_mode) {
        ROS_ERROR("[IMU Calibration]: SetParamsBiasInstability");
    }

    // find global minimum in simulated Allan variance curve:
    double min_covariance[NUM_FIELDS] = {
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max()
    };
    for (const Point &point: params_.curve_smoothed.point) {
        for (int field = WX; field < NUM_FIELDS; ++field) {
            if (min_covariance[field] > point.covariance[field]) {
                min_covariance[field] = point.covariance[field];
            }
        }
    }

    // random walk, standard deviation:
    for (int field = WX; field < NUM_FIELDS; ++field) {
        params_.bias_instability[field] = min_covariance[field];
    }
}

void AllanVariance::SetParams(void) {
    if (config_.debug_mode) {
        ROS_ERROR("[IMU Calibration]: SetParams");
    }

    // number of observations:
    params_.num_observations = state_.data.size();
    // number of clusters:
    params_.num_clusters = state_.averaging_factor.size();

    for (int field = WX; field < NUM_FIELDS; ++field) {
        // get initial estimation using least square:
        Eigen::VectorXd init_params = GetInitialEstimation(static_cast<Field>(field));
        // refine estimation using ceres:
        Eigen::VectorXd final_params = GetRefinedEstimation(static_cast<Field>(field), init_params);

        // quantization noise:
        params_.Q[field] = sqrt(final_params(0) * final_params(0) / 3.0);
        // angle random walk:
        params_.N[field] = sqrt(final_params(1) * final_params(1));
        // bias instability, constant -- 2*np.log(2) / np.pi:
        params_.B[field] = sqrt(final_params(2) * final_params(2) * 2.266180070913597);
        // rate random walk:
        params_.K[field] = sqrt(final_params(3) * final_params(3) * 3.0);
        // random ramp:
        params_.R[field] = sqrt(final_params(4) * final_params(4) * 2.0);

        if (config_.debug_mode) {
            ROS_ERROR(
                "[IMU Calibration]: SetParams: field %d, init params %f %f %f", 
                field, 
                init_params(0), init_params(1), init_params(2)
            );
            ROS_ERROR(
                "[IMU Calibration]: SetParams: field %d, refined params %f %f %f", 
                field, 
                final_params(0), final_params(1), final_params(2)
            );
        }
    }

    // generate simulated allan variance curve:
    SetParamsCurveSmoothed();

    // set bias instability:
    SetParamsMeasurementNoise();
    SetParamsRandomWalk();
    SetParamsBiasInstability();
}

}  // namespace allan_variance

}  // namespace calibrator

}  // namespace imu