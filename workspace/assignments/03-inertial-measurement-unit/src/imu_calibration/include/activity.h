#ifndef IMU_CALIBRATION_ACTIVITY_H
#define IMU_CALIBRATION_ACTIVITY_H

#include <string>
#include <unordered_map>
#include <map>

#include <sensor_msgs/Imu.h>
#include <boost/thread/thread.hpp>
#include <actionlib/server/simple_action_server.h>

#include "node_constants.h"

#include "allan_variance.h"

#include "imu_calibration/CalibrateIMUAction.h"
#include "imu_calibration/GetAllanVarianceAnalysisResult.h"


namespace imu_calibration {

struct Config {
    bool debug_mode;
    int max_playback_rate;

    std::string topic_name;
    std::string device_name;
    std::string output_path;

    int min_collection_time_in_mins;
    int max_num_clusters;
};

struct State {
    double timestamp_start;
    allan_variance::AllanVariance estimator;
    boost::thread* thread;
    imu_calibration::CalibrateIMUFeedback feedback;
    imu_calibration::CalibrateIMUResult result;

    State(std::string name, int max_num_clusters) : estimator(name, max_num_clusters), thread(nullptr){}
};

namespace com {
  class Activity {
  public:
    template <class T, class M>
    void Subscribe(const std::string& topic_name, void (T::*fp)(M), T* obj, int queue_size=5) {
      ros::SubscribeOptions ops;
      ops.template initByFullCallbackType<M>(topic_name, queue_size,
                                            boost::bind(fp, obj, _1));
      ops.transport_hints = ros::TransportHints();
      subscribers_[topic_name] = nh_.subscribe(ops);
    }

    template <class T, class MReq, class MRes>
    void AdvertiseService(const std::string& service_name,
                          bool (T::*fp)(MReq&, MRes&), T* obj) {
      ros::AdvertiseServiceOptions ops;
      ops.template init<MReq, MRes>(service_name, boost::bind(fp, obj, _1, _2));
      service_servers_[service_name] = nh_.advertiseService(ops);
    }

    template <class T>
    ros::Publisher Advertise(const std::string& topic_name) {
      return nh_.advertise<T>(topic_name, 5);
    }
    void ShutdownSubscriber(const std::string& topic_name) {
      if (subscribers_.count(topic_name)) {
        subscribers_.at(topic_name).shutdown();
      }
    }
    virtual void Init() = 0;

  protected:
    ros::NodeHandle nh_;
    std::unordered_map<std::string, ros::Subscriber> subscribers_;
    std::unordered_map<std::string, ::ros::ServiceServer> service_servers_;
  };
}

class Activity : public com::Activity {
public:
    Activity();
    ~Activity();

    void Init(void);
    void CheckGoal(void);
private:
    void LaunchSubscribers(int queue_size);
    void ShutdownSubscribers(void);
    
    void SetMinCollectionTimeCB(void); 
    void PreemptCB(void);
    void ImuCB(const sensor_msgs::ImuConstPtr& msg);
    void DoEstimate(void);

    bool GetAllanVarianceAnalysisResult(    
        imu_calibration::GetAllanVarianceAnalysisResult::Request& req,
        imu_calibration::GetAllanVarianceAnalysisResult::Response& res
    );

    ros::NodeHandle private_nh_;
    actionlib::SimpleActionServer<imu_calibration::CalibrateIMUAction> as_;

    Config config_;
    State state_;
};

}  // namespace imu_calibration

#endif  // IMU_CALIBRATION_ACTIVITY_H