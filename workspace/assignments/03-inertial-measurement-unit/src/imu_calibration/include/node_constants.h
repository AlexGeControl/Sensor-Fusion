#ifndef IMU_CALIBRATOR_CONSTANTS_H
#define IMU_CALIBRATOR_CONSTANTS_H


namespace imu {

namespace calibrator {
    // actions:
    constexpr char kCalibrateIMUActionName[] = "/imu/calibrator/action";
    // services:
    constexpr char kGetIMUCalibrationResults[] = "/imu/calibrator/get_results";
} // namespace calibrator

} // namespace imu

#endif // IMU_CALIBRATOR_CONSTANTS_H
