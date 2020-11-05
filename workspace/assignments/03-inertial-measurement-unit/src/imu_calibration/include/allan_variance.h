#ifndef IMU_CALIBRATOR_ALLAN_VARIANCE_H
#define IMU_CALIBRATOR_ALLAN_VARIANCE_H

#include <math.h>
#include <string>
#include <vector>
#include <map>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <geometry_msgs/Vector3.h>

namespace imu {

namespace calibrator {

namespace allan_variance {

/*
    estimator configuration
 */
struct Config {
    bool debug_mode;
    std::string name;
    int max_num_clusters;
};

enum Field {
    WX = 0,
    WY,        
    WZ,
    AX,
    AY,        
    AZ,
    NUM_FIELDS
};

struct Data {
    double time;

    double value[NUM_FIELDS];
};

/*
    allan variance curve
 */
struct Point {
    double tau;
    double covariance[NUM_FIELDS];
};

struct Curve {
    std::vector<Point> point;
};

/*
    estimator state
 */
struct State {
    double tau;

    std::vector<Data> data;
    std::map<Field, std::vector<double>> theta; 
    
    std::vector<int> averaging_factor;
    size_t num_clusters_built;
    Curve curve_observed;

    State() {
        tau = 0.0;

        data.clear();
        // initialize theta:
        for (int field = WX; field < NUM_FIELDS; ++field) {
            theta.insert(
                std::map<Field, std::vector<double>>::value_type (static_cast<Field>(field), std::vector<double>())
            );
        }

        // initialize curve:
        averaging_factor.clear();
        num_clusters_built = 0;
        curve_observed.point.clear();
    }

    void Reset(void) {
        tau = 0.0;

        data.clear();
        // reset theta:
        for (int field = WX; field < NUM_FIELDS; ++field) {
            theta.at(static_cast<Field>(field)).clear();
        }

        // reset curve:
        averaging_factor.clear();
        num_clusters_built = 0;
        curve_observed.point.clear();
    }
};

struct Params {
    // number of observations:
    size_t num_observations;
    // number of clusters:
    size_t num_clusters;

    // quantization noise:
    double Q[NUM_FIELDS];
    // angle random walk:
    double N[NUM_FIELDS];
    // bias instability:
    double B[NUM_FIELDS];
    // rate random walk:
    double K[NUM_FIELDS];
    // drift rate ramp:
    double R[NUM_FIELDS];

    Curve curve_smoothed;

    double measurement_noise[NUM_FIELDS];
    double random_walk[NUM_FIELDS];
    double bias_instability[NUM_FIELDS];

    void Reset(void) {
        curve_smoothed.point.clear();
    }
};

/*
    parameter estimation
 */
struct Error
{
    Error(const double& tau, const double& covariance): tau_(tau), covariance_(covariance) {
    }

    template< typename T >
    T Log10( T src ) const
    {
        return ( log( src ) ) / ( log( 10 ) );
    }

    template< typename T >
    T GetCovariance(T Q, T N, T B, T K, T R, T tau) const
    {
        // clang-format off
        return  (
            ( Q*Q / (tau*tau) ) + 
            ( N*N / tau ) + 
            ( B*B ) + 
            ( K*K * tau ) + 
            ( R*R * (tau*tau) )
        );
        // clang-format on
    }

    template< typename T >
    bool operator()(const T* const params, T* residuals) const
    {
        // parse parameters:
        T Q   = T( params[0] );
        T N   = T( params[1] );
        T B   = T( params[2] );
        T K   = T( params[3] );
        T R   = T( params[4] );
        T tau = T( tau_ );

        // get prediction:
        T covariance = GetCovariance(Q, N, B, K, R, tau);
        // error in log10 space -- here add bias for both value to prevent log10(0) to give -inf:
        T error      = T(Log10(covariance + bias_)) - T(Log10(covariance_ + bias_));

        // set residual:
        residuals[0] = error;

        return true;
    }

    double tau_;
    double covariance_;
    double bias_ = std::numeric_limits<double>::min();
};

class AllanVariance {
public:
    AllanVariance(bool debug_mode, std::string name, int max_num_clusters);
    ~AllanVariance(void);

    void Reset(void);
    void Add(
        double time, 
        const geometry_msgs::Vector3 &angular_velocity, 
        const geometry_msgs::Vector3 &linear_acceleration 
    );
    void Estimate(void);

    void SetDebugMode(bool debug_mode) { config_.debug_mode = debug_mode; }
    void SetName(std::string name) { config_.name = name; }
    void SetMaxNumClusters(int max_num_clusters) { config_.max_num_clusters = max_num_clusters; }

    // get latest observation timestamp:
    double GetT(void) { 
        if (state_.data.empty()) { return -1.0; }
        return state_.data.back().time; 
    }
    // get allan variance curve building progress:
    double GetAllanCurveBuildingProgress(void) {
        return 100.0 * state_.num_clusters_built / state_.averaging_factor.size();
    }
    // get quantization noise:
    double GetQ(Field field) { return params_.Q[field]; }
    // get angle random walk:
    double GetN(Field field) { return params_.N[field]; }
    // get bias instability:
    double GetB(Field field) { return params_.B[field]; }
    // get rate random walk:
    double GetK(Field field) { return params_.K[field]; }
    // get rate ramp:
    double GetR(Field field) { return params_.R[field]; }
    // get measurement noise:
    double GetMeasurementNoise(Field field) { return params_.measurement_noise[field]; }
    // get random walk:
    double GetRandomWalk(Field field) { return params_.random_walk[field]; }
    // get bias instability:
    double GetBiasInstability(Field field) { return params_.bias_instability[field]; }

    // show IMU params:
    void WriteIMUCalibrationResult(const std::string &output_filename);
private:
    void SetStateAveragingFactor(void);
    void SetStateCurveObserved(void);
    void SetState(void);

    double GetCovariance(Field field, const double &tau);
    // fast estimate using least square:
    Eigen::VectorXd GetInitialEstimation(Field field);
    // refined estimate using ceres:
    Eigen::VectorXd GetRefinedEstimation(Field field, const Eigen::VectorXd &params);
    void SetParamsCurveSmoothed(void);
    void SetParamsMeasurementNoise(void);
    void SetParamsRandomWalk(void);
    void SetParamsBiasInstability(void);
    void SetParams(void);

    Config config_;
    State state_;
    Params params_;
};

}  // namespace allan_variance

}  // namespace calibrator

}  // namespace imu

#endif  // IMU_CALIBRATOR_ALLAN_VARIANCE_H