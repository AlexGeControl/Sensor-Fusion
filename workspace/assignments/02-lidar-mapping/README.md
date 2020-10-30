# Sensor Fusion: Lidar Mapping -- 多传感器融合定位: 基于地图的定位

This is the solution of Assignment 02 of Sensor Fusion from [深蓝学院](https://www.shenlanxueyuan.com/course/261).

深蓝学院从多传感器融合定位第2节Lidar Mapping答案. 版权归深蓝学院所有. 请勿抄袭.

---

## Problem Statement

---

### 1. 闭环修正及精度评价

提供的工程框架中, 已经给出了闭环的流程和实现, 但是是基于ICP的, 这是在已知粗略位姿情况下的实现方式. 在未知粗略位姿情况下, 闭环检测的实现难度会加大. 要求使用前面讲过的`Scan Context`, 实现此功能, 并和已有的图优化功能完成对接, 实现`Loop Closure`修正. 并最终给出修正前后的轨迹精度对比.

#### ANS

基于`Scan Context`的`Loop Closure Detection`核心逻辑如下. 此处使用`Scan Context`产生所需的`Loop Closure Proposal`, `GNSS/IMU`提供的粗略位姿估计仅用于检查Scan Context提供回环中关键帧的距离.

```c++
    // only perform loop closure detection for every skip_num key frames:
    if (++skip_cnt < skip_num)
        return false;

    // generate loop-closure proposal using scan context match:
    std::pair<int, float> proposal = scan_context_manager_ptr_->DetectLoopClosure();
    const int proposed_key_frame_id = proposal.first;
    const float proposed_yaw_change = proposal.second;

    // check proposal validity:
    if (ScanContextManager::NONE == proposed_key_frame_id) {
        return false;
    }

    // check RTK position difference:
    const KeyFrame &current_key_frame = all_key_gnss_.back();
    const KeyFrame &proposed_key_frame = all_key_gnss_.at(proposed_key_frame_id);

    Eigen::Vector3f translation = (
        current_key_frame.pose.block<3, 1>(0, 3) - proposed_key_frame.pose.block<3, 1>(0, 3)
    );
    float key_frame_distance = translation.norm();

    // update detection interval:
    skip_cnt = 0;
    skip_num = static_cast<int>(key_frame_distance);
    if (key_frame_distance > detect_area_) {
        skip_num = std::max((int)(key_frame_distance / 2.0), loop_step_);
        return false;
    } else {
        key_frame_index = proposed_key_frame_id;
        yaw_change_in_rad = proposed_yaw_change;

        skip_num = loop_step_;
        return true;
    }
```

核心功能的实现, 请点击下述链接:

* [Scan Context for Loop Closure Detection](src/lidar_localization/src/mapping/loop_closing/loop_closing.cpp)

* [Scan Context Manager Implementation](src/lidar_localization/src/models/scan_context_manager/scan_context_manager.cpp)

`Scan Context Loop Closure`修正后的轨迹估计如下图所示:

<img src="doc/images/01-optimized-trajectory-with-loop-closure.png" width="%100" alt="Trajectory Estimation using Scan Context">

使用`Scan Context`修正前后, 轨迹估计误差的对比如下所示:

GNSS/IMU Proposal          |Scan Context Proposal
:-------------------------:|:-------------------------:
![EVO APE GNSS/IMU](doc/images/01-error-map-scan-context.png)  |  ![EVO APE ICP](doc/images/01-error-map-scan-context.png)

|  Prop. |    GNSS/IMU   |  Scan Context |
|:------:|:-------------:|:-------------:|
|   max  |    0.514676   |    0.514676   |
|  mean  |    0.095825   |    0.095825   |
| median |    0.050754   |    0.050754   |
|   min  |    0.008929   |    0.008929   |
|  rmse  |    0.133023   |    0.133023   |
|   sse  |   33.620606   |   33.620606   |
|   std  |    0.092264   |    0.092264   |

由上述结果可知, `Scan Context`的使用, `可以显著提升轨迹估计精度`.

---

### 2. 位姿初始化

提供的工程框架中, 已经给出了位姿初始化功能, 但是是在起始位置的, 并且是基于已知粗略位姿的. 要求实现`地图中任意位置的位姿初始化`, 可以从三种难度等级中任选一种, 难度越高, 得分越高.