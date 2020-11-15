# Sensor Fusion: IMU-Lidar-GNSS Fusion -- 多传感器融合定位: 基于滤波的融合定位

This is the solution of Assignment 04 of Sensor Fusion from [深蓝学院](https://www.shenlanxueyuan.com/course/261).

深蓝学院从多传感器融合定位第4节IMU-Lidar-GNSS Fusion for Localization答案. 版权归深蓝学院所有. 请勿抄袭.

---

## Problem Statement

---

### 1. 基于滤波的融合定位

任选一种滤波模型和方法, 在`KITTI`数据集上, 实现基于`IMU`以及`点云地图匹配`的融合定位

#### ANS

此处选用`基于误差信息的Kalman Filtering`进行融合定位. 解决方案的架构图如下:

<img src="doc/images/01-error-based-kalman-filtering--architect.png" alt="Trajectory Estimation using Scan Context" width="100%" />

算法的`理论推导`以及`针对KITTI Road Test Data`的`伪代码实现`参考 [here](doc/derivations)

为了产生`未降频的IMU测量值`:

1. 下载`extract.zip`
2. 解压后, 将其中的`oxts`替换`sync`中的`oxts`
3. 修复`extract oxts`测量值的`timestamp`异常. 原始`timestamp`差分的统计描述如下:

    ```bash
    count                       45699
    mean       0 days 00:00:00.010318
    std        0 days 00:00:00.022812
    min      -1 days +23:59:59.949989
    25%        0 days 00:00:00.009967
    50%        0 days 00:00:00.009996
    75%        0 days 00:00:00.010025
    max        0 days 00:00:01.919595
    ```

    使用[scripts](src/lidar_localization/scripts/generate_100hz_oxts.py)中的脚本进行修复, 修复后的`timestamp`差分统计描述如下:

    ```bash
    count                     50244
    mean     0 days 00:00:00.009384
    std      0 days 00:00:00.006092
    min             0 days 00:00:00
    25%      0 days 00:00:00.009953
    50%      0 days 00:00:00.009992
    75%      0 days 00:00:00.010021
    max      0 days 00:00:00.519616
    ```

4. 修改`kitti2bag`源代码, 将`IMU` `Linear Acceleration / Angular Velocity`测量值的坐标系改为`Body Frame(xyz)` [here](src/lidar_localization/scripts/kitti2bag.py#L39)
    
    ```python
    def save_imu_data(bag, kitti, imu_frame_id, topic):
    print("Exporting IMU")
    for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
        q = tf.transformations.quaternion_from_euler(oxts.packet.roll, oxts.packet.pitch, oxts.packet.yaw)
        imu = Imu()
        imu.header.frame_id = imu_frame_id
        imu.header.stamp = rospy.Time.from_sec(float(timestamp.strftime("%s.%f")))
        imu.orientation.x = q[0]
        imu.orientation.y = q[1]
        imu.orientation.z = q[2]
        imu.orientation.w = q[3]
        # change here:
        imu.linear_acceleration.x = oxts.packet.ax
        imu.linear_acceleration.y = oxts.packet.ay
        imu.linear_acceleration.z = oxts.packet.az
        imu.angular_velocity.x = oxts.packet.wx
        imu.angular_velocity.y = oxts.packet.wy
        imu.angular_velocity.z = oxts.packet.wz
        bag.write(topic, imu, t=imu.header.stamp)
    ```
5. 然后运行`kitti2bag`, 产生用于`LIO/VIO Filtering/Graph Optimization`的ROS Bag

6. 为了提高`lidar-IMU-GNSS`配准的精度, 方便`evo`的精度评估:

    * 删除`sync.bag`中的`\tf_static`, `\tf`
    * 保留`extract.bag`中的`\tf_static`, `\tf`和`/kitti/oxts/imu`, 并将`/kitti/oxts/imu`重命名为`/kitti/oxts/imu/extract`
    * 合并上述两生成bag, 作为最终的`synced.bag`

在`kitti_2011_10_03_drive_0027_synced`上得到的ROS Bag Info如下:

```bash
$ rosbag info kitti_2011_10_03_drive_0027_synced.bag

path:        kitti_2011_10_03_drive_0027_synced.bag
version:     2.0
duration:    7:51s (471s)
start:       Oct 03 2011 20:55:34.93 (1317646534.93)
end:         Oct 03 2011 21:03:26.01 (1317647006.01)
size:        24.1 GB
messages:    192006
compression: none [18243/18243 chunks]
types:       geometry_msgs/TwistStamped [98d34b0043a2093cf9d9345ab6eef12e]
             sensor_msgs/CameraInfo     [c9a58c1b0b154e0e6da7578cb991d214]
             sensor_msgs/Image          [060021388200f6f0f447d0fcd9c64743]
             sensor_msgs/Imu            [6a62c6daae103f4ff57a132d6f95cec2]
             sensor_msgs/NavSatFix      [2d3a8cd499b9b4a0249fb98fd05cfa48]
             sensor_msgs/PointCloud2    [1158d486dd51d683ce2f1be655c3c181]
             tf2_msgs/TFMessage         [94810edda583a504dfda3829e70d7eec]
topics:      /kitti/camera_color_left/camera_info     4544 msgs    : sensor_msgs/CameraInfo    
             /kitti/camera_color_left/image_raw       4544 msgs    : sensor_msgs/Image         
             /kitti/camera_color_right/camera_info    4544 msgs    : sensor_msgs/CameraInfo    
             /kitti/camera_color_right/image_raw      4544 msgs    : sensor_msgs/Image         
             /kitti/camera_gray_left/camera_info      4544 msgs    : sensor_msgs/CameraInfo    
             /kitti/camera_gray_left/image_raw        4544 msgs    : sensor_msgs/Image         
             /kitti/camera_gray_right/camera_info     4544 msgs    : sensor_msgs/CameraInfo    
             /kitti/camera_gray_right/image_raw       4544 msgs    : sensor_msgs/Image         
             /kitti/oxts/gps/fix                      4544 msgs    : sensor_msgs/NavSatFix     
             /kitti/oxts/gps/vel                      4544 msgs    : geometry_msgs/TwistStamped
             /kitti/oxts/imu                          4544 msgs    : sensor_msgs/Imu           
             /kitti/oxts/imu/extract                 45826 msgs    : sensor_msgs/Imu           
             /kitti/velo/pointcloud                   4544 msgs    : sensor_msgs/PointCloud2   
             /tf                                     45826 msgs    : tf2_msgs/TFMessage        
             /tf_static                              45826 msgs    : tf2_msgs/TFMessage

```

基于`Eigen`与`Sophus`的`Error-State Kalman Fusion`实现参考[here](src/lidar_localization/src/models/kalman_filter/kalman_filter.cpp)

`IMU-Lidar Error-State Kalman Fusion Odometry`与`GNSS Groud Truth`的对比如下图所示. 其中`黄色`为`GNSS Groud Truth`, `红色`为`Lidar Odometry`, `蓝色`为`IMU-Lidar Fusion Odometry`:

<img src="doc/images/01-IMU-lidar-fusion.png" alt="IMU-Lidar Fusion v.s. GNSS" width="100%">

<img src="doc/images/01-IMU-lidar-fusion-micro.png" alt="IMU-Lidar Fusion v.s. GNSS Micro" width="100%">

可通过如下`ROS Service Call`, 比较融合前后的Odometry: 

```bash
# set up session:
source install/setup.bash
# save odometry:
rosservice call /save_odometry "{}"
# run evo evaluation:
# a. laser:
evo_ape kitti ground_truth.txt laser.txt -r full --plot --plot_mode xy
# b. fused:
evo_ape kitti ground_truth.txt fused.txt -r full --plot --plot_mode xy
```

两者的KPI比较参照下表. 

在`2011_10_03_drive_0027_extract`上, 两者的估计性能相近, `IMU-Lidar Fusion`的估计精度略优.

Lidar Only                 |IMU-Lidar Fusion
:-------------------------:|:-------------------------:
![EVO Lidar Only](doc/images/01-evo--lidar-only-map-plot.png)  |  ![EVO IMU-Lidar Fusion](doc/images/01-evo--imu-lidar-fusion-map-plot.png)

|  Algo. |  Lidar Only   |  IMU-Lidar    |
|:------:|:-------------:|:-------------:|
|   max  |   1.059857    |    1.035433   |
|  mean  |   0.228137    |    0.232957   |
| median |   0.160143    |    0.165522   |
|   min  |   0.015406    |    0.015572   |
|  rmse  |   0.284854    |    0.288054   |
|   sse  |  356.212471   |  364.259559   |
|   std  |   0.170574    |    0.169428   |	

---

### 2. GNSS/IMU融合分析

推导组合导航(GNSS + IMU)的滤波模型. 要求:

* 对静止, 匀速, 转向, 加减速等不同运动状态下各状态量的可观测性和可观测度进行分析.

* 使用第三章所述数据仿真软件, 产生对应运动状态的数据, 进行Kalman滤波.

* 统计Kalman滤波中各状态量的收敛速度和收敛精度, 并与可观测度分析的结果汇总比较.

#### ANS
