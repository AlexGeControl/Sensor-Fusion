#ifndef IMU_CALIBRATION_NODE_CONSTANTS_H_
#define IMU_CALIBRATION_NODE_CONSTANTS_H_
namespace imu_calibration
{
    // actions:
    constexpr char kCalibrateIMUAction[] = "/calibration/imu/action";
    // services:
    constexpr char kGetIMUCalibrationResults[] = "/calibration/imu/get_results";
} // namespace imu_calibration

#endif // IMU_CALIBRATION_NODE_CONSTANTS_H_
