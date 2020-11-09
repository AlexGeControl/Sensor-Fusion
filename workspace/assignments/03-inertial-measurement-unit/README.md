# Sensor Fusion: Inertial Measurement Unit -- 多传感器融合定位: 惯性导航器件

This is the solution of Assignment 03 of Sensor Fusion from [深蓝学院](https://www.shenlanxueyuan.com/course/261).

深蓝学院多传感器融合定位第3节Inertial Measurement Unit答案. 版权归深蓝学院所有. 请勿抄袭.

---

## Problem Statement

---

### 1. Simulate IMU Measurement and Perform Allan Variance Analysis on Simulated Data
### 1. 仿真IMU数据, 并进行Allan方差分析

#### ANS

首先, 使用`gnss-ins-sim ROS wrapper package`, 产生`Allan Variance analysis`所需的`ROS Bag`:

```bash
# build
catkin config --install && catkin build gnss_ins_sim
# run:
source install/setup.bash
roslaunch gnss_ins_sim gnss_ins_sim_recorder.launch
```

若使用默认配置, 可在Docker环境`/workspace/data/gnss_ins_sim`路径下发现生成的`ROS Bag`.

接着, 使用`imu_calibration`包, 进行`Allan Variance analysis`:

```bash
# build
catkin config --install && catkin build imu_calibration
# run:
source install/setup.bash
roslaunch imu_calibration imu_calibration.launch
```

若使用默认配置, 可在Docker环境`/workspace/data/gnss_ins_sim`路径下发现生成的`imu_calibration_results`

#### Results 

对`gnss-ins-sim`中的`low-accuracy`模型, `Allan Variance analysis`得到的`ARW/VRW(参数N)`估计结果与真值的对比如下. 结果显示`标定算法能够有效估计IMU的噪声参数`:

| ARW/VRW | Ground Truth | Allan Variance Estimation |
|:-------:|:------------:|:-------------------------:|
|  gyro_x | 2.181662e-04 |        2.174482e-04       |
|  gyro_y | 2.181662e-04 |        2.190593e-04       |
|  gyro_z | 2.181662e-04 |        2.209243e-04       |
| accel_x | 8.333333e-04 |        8.226509e-04       |
| accel_y | 8.333333e-04 |        8.454541e-04       |
| accel_z | 8.333333e-04 |        8.345325e-04       |

所获得的`Allan Variance curves`如下图所示:

Allan Variance Curve, Gyro |Allan Variance Curve, Accel
:-------------------------:|:-------------------------:
![Allan Variance Curve, Gyro](doc/01-allan-variance-curve--gyro.png)  |  ![Allan Variance Curve, Accel](doc/01-allan-variance-curve--acc.png)

--- 

### 2. 设计一种转台旋转方案, 并基于仿真数据, 进行内参求解的验证

#### ANS

`转台旋转方案`设计如下. 方案包含`两个阶段`:

* `第一阶段--陀螺仪刻度系数K与安装误差S求解` 
    * 使转台分别沿`IMU`系`+-X`, `+-Y`, `+-Z`三个轴正反方向做角速度幅值, 时间相同的定轴旋转, 标定`陀螺仪`的`刻度系数`与`安装误差`.
* `第二阶段--陀螺仪零偏误差Epsilon求解, 以及加速度计刻度系数K, 安装误差S以及零偏误差Epsilon的求解`
    * 使转台分别绕`IMU`系`Yaw`, `Pitch`, `Roll`三个轴旋转`+-90`度, 然后静止一段时间, 标定`陀螺仪`的`零偏误差`, 以及`加速度计`的`刻度系数`， `安装误差`以及`零偏误差`.

上述方案对应的`gnss-ins-sim`定义参见[here](src/gnss_ins_sim/config/motion_def/deterministic_error_calib_gyro.csv)

对应算法的`Python`实现参见[here](src/imu_calibration/scripts/estimate_deterministic_error.py#L125)

标定结果的真值与估计值对比如下. 结果显示`标定算法能够有效估计IMU的确定性误差参数`:

* Ground Truth:

    ```python
    # GNSS/INS sim config:
    imu_err = {
        # 1. gyro:
        # a. random noise:
        # gyro angle random walk, deg/rt-hr
        'gyro_arw': np.array([0.00, 0.00, 0.00]),
        # gyro bias instability, deg/hr
        'gyro_b_stability': np.array([0.0, 0.0, 0.0]),
        # gyro bias isntability correlation time, sec
        # 'gyro_b_corr': np.array([100.0, 100.0, 100.0]),
        # b. deterministic error:
        'gyro_b': np.array([36.00, 36.00, 36.00]),
        'gyro_k': np.array([0.98, 0.98, 0.98]),
        'gyro_s': np.array([0.01, 0.01, 0.01, 0.01, 0.01, 0.01]),
        # 2. accel:
        # a. random noise:
        # accel velocity random walk, m/s/rt-hr
        'accel_vrw': np.array([0.05, 0.05, 0.05]),
        # accel bias instability, m/s2
        'accel_b_stability': np.array([2.0e-4, 2.0e-4, 2.0e-4]),
        # accel bias isntability correlation time, sec
        'accel_b_corr': np.array([100.0, 100.0, 100.0]),
        # b. deterministic error:
        'accel_b': np.array([0.01, 0.01, 0.01]),
        'accel_k': np.array([0.98, 0.98, 0.98]),
        'accel_s': np.array([0.01, 0.01, 0.01, 0.01, 0.01, 0.01]),
        # 3. mag:
        'mag_si': np.eye(3) + np.random.randn(3, 3)*0.0, 
        'mag_hi': np.array([10.0, 10.0, 10.0])*0.0,
        'mag_std': np.array([0.1, 0.1, 0.1])
    }
    ```

    ```json
    {
        "gyro": {
            "scale": [
                0.980, 0.010, 0.010, 
                0.010, 0.980, 0.010, 
                0.010, 0.010, 0.980
            ], 
            "bias": {
                "x": 36.000,
                "y": 36.000, 
                "z": 36.000
            }
        },
        "accel": {
            "scale": [
                0.980, 0.010, 0.010, 
                0.010, 0.980, 0.010, 
                0.010, 0.010, 0.980
            ], 
            "bias": {
                "x": 0.010,
                "y": 0.010, 
                "z": 0.010
            }
        }
    }
    ```

* Estimated Result(For raw output click [here](doc/02-separated-calibration-results.json)):

    ```json
    {
        "gyro": {
            "scale": [
                0.980, 0.010, 0.010, 
                0.010, 0.980, 0.010, 
                0.010, 0.010, 0.980
            ], 
            "bias": {
                "y": 35.999, 
                "x": 35.999, 
                "z": 35.999
            }
        }, 
        "accel": {
            "scale": [
                0.980, 0.010, 0.010, 
                0.010, 0.980, 0.010, 
                0.010, 0.010, 0.980
            ], 
            "bias": {
                "y": 0.010, 
                "x": 0.010, 
                "z": 0.010
            }
        }
    }
    ```

---

### 3. 推导基于Levenberg-Marquardt方法, 进行加速度计和陀螺仪内参估计的优化过程. 按照课程中讲述的模型, 修改参考代码, 使用仿真数据, 验证标定算法的正确性

---

### 4. 对一组数据进行惯性导航解算验证, 要求:

* 数据可以使用仿真数据,也可以使用KITTI数据集中的数据, 只不过使用KITTI数据时, 要修改kitti2bag的代码,把`IMU`的输出频率恢复成`100Hz`,它的原版程序在生成bag时对数据做了降频.

* 由于MEMS惯导误差发散快, 导航时间不用太长, 几分钟即可. 如果想验证高精度惯导随时间发散的现象, 可用仿真数据生成.

* 解算时尽量对比验证`角增量方法`和`旋转矢量方法`的区别. 由于定轴转动下, 二者没有区别, 因此若要验证该现象, 数据要运动剧烈些.
