# Sensor Fusion: Graph Optimization for Lidar Mapping -- 多传感器融合定位: 基于图优化的建图

This is the solution of Assignment 06 of Sensor Fusion from [深蓝学院](https://www.shenlanxueyuan.com/course/261).

深蓝学院从多传感器融合定位第6节Graph Optimization for Lidar Mapping答案. 版权归深蓝学院所有. 请勿抄袭.

---

## Problem Statement

---

## 1. 在KITTI上, 实现基于IMU的预积分融合激光建图

### ANS

#### LIO-G2O Implementation

首先实现基于G2O的LIO Mapping. 包括:

* IMU Pre-Integration [here](src/lidar_localization/src/models/pre_integrator/imu_pre_integrator.cpp#L178)
* G2O Vertex / Edge for IMU Pre-Integration
    * LIO Key Frame Vertex [here](src/lidar_localization/include/lidar_localization/models/graph_optimizer/g2o/vertex/vertex_prvag.hpp)
    * IMU Pre-Integration Edge [here](src/lidar_localization/include/lidar_localization/models/graph_optimizer/g2o/edge/edge_prvag_imu_pre_integration.hpp)

然后实现`LIO Backend`, 将`IMU Pre-Integration`集成至Mapping框架.

#### Results & Analysis

`LIO Mapping`得到的轨迹估计如下图所示. 其中红色为`Lidar Frontend`的估计, 绿色为`LIO Mapping`的估计. 优化后的轨迹与黄色的`GNSS Ground Truth`重合较好:

<img src="doc/images/01-optimized-trajectory-using-lio.png" width="%100" alt="Trajectory Estimation using LIO">

使用`IMU Pre-Integration`修正前后, 轨迹估计误差的对比如下所示:

Before                     |After
:-------------------------:|:-------------------------:
![Before](doc/images/01-evo-lidar-frontend--time-series-plot.png)  |  ![After](doc/images/01-evo-optimized--time-series-plot.png)
![Before](doc/images/01-evo-lidar-frontend--map-plot.png)  |  ![After](doc/images/01-evo-optimized--map-plot.png)

|  Prop. |     Before    |     After     |
|:------:|:-------------:|:-------------:|
|   std  |   15.656619   | **15.571682** |

由上述结果可知, `IMU Pre-Integration`的使用, `可以提升轨迹估计精度`.

在`LIO Mapping`得到的地图上

* 系统基于`ESKF`与`IEKF`的融合定位能够稳定运行
* 观测到的点云与地图中的点云重合度良好

测试的结果如下图所示. 结果显示`LIO Mapping 建图质量良好`.

![Localiztion & Map Integrity Demo](doc/images/01-localization-demo-a.png)
![Localiztion & Map Integrity Demo](doc/images/01-localization-demo-b.png)
![Localiztion & Map Integrity Demo](doc/images/01-localization-demo-c.png)
![Localiztion & Map Integrity Demo](doc/images/01-localization-demo-d.png)
![Localiztion & Map Integrity Demo](doc/images/01-localization-demo-e.png)
![Localiztion & Map Integrity Demo](doc/images/01-localization-demo-f.png)

---

## 2. 基于Odometer的预积融合分方法

推导基于编码器的预积分方法, 包括:

1. 预积分模型
2. 预积分残差的设计
3. 预积分方差的递推
4. 预积分对各状态量扰动的雅可比

推导思路可参考论文 `VINS on Wheels` [here](http://mars.cs.umn.edu/papers/KejianWu_VINSonWheels.pdf)

### ANS

完整推导参考 [here](doc/derivation/odo-pre-integration.pdf)

此处将关键结论整合如下

#### 预积分模型

<img src="doc/images/02-a-pre-integration-update.png" alt="Odo Pre Integration, Model" width="100%" />

#### 预积分残差的设计

<img src="doc/images/02-c-residual.png" alt="Odo Pre Integration, Residual" width="100%" />

#### 预积分方差的递推

<img src="doc/images/02-b-error-propagation.png" alt="Odo Pre Integration, Error Propagation" width="100%" />

#### 预积分对各状态量扰动的雅可比

TBD

---

## 3. 在KITTI上, 实现基于IMU-Odometer的预积分融合激光建图

### ANS

此处使用组合导航估计的`w`和`v`, 作为`Odometer`的观测量, 尝试在KITTI数据集上实现`LIO on Wheels`

首先扩展基于G2O的LIO Mapping. 增加`Odo Pre-Integration`的实现, 包括:

* Odo Pre-Integration [here](src/lidar_localization/src/models/pre_integrator/odo_pre_integrator.cpp#127)
* G2O Edge for IMU Pre-Integration)
    * Odo Pre-Integration Edge [here](src/lidar_localization/include/lidar_localization/models/graph_optimizer/g2o/edge/edge_prvag_odo_pre_integration.hpp)

然后扩展`LIO Backend`, 将`Odo Pre-Integration`集成至Mapping框架.

#### Results & Analysis

`LIO Mapping with Odo Pre-Integration`得到的轨迹估计如下图所示. 其中红色为`Lidar Frontend`的估计, 绿色为`LIO Mapping with Odo Pre-Integration`的估计. 优化后的轨迹与黄色的`GNSS Ground Truth`重合较好:

<img src="doc/images/01-optimized-trajectory-using-lio.png" width="%100" alt="Trajectory Estimation using LIO with Odo Pre-Integration">

使用`Odo Pre-Integration`修正前后, 轨迹估计误差的对比如下所示:

Before                     |After
:-------------------------:|:-------------------------:
![Before](doc/images/03-evo-lidar-frontend--time-series-plot.png)  |  ![After](doc/images/03-evo-optimized--time-series-plot.png)
![Before](doc/images/03-evo-lidar-frontend--map-plot.png)  |  ![After](doc/images/03-evo-optimized--map-plot.png)

|  Prop. |     Before    |     After     |
|:------:|:-------------:|:-------------:|
|   std  |    7.316777   |  **7.240568** |

由上述结果可知, `Odo Pre-Integration`的使用, `可以提升轨迹估计精度`.

在`LIO Mapping with Odo Pre-Integration`得到的地图上

* 系统基于`ESKF`与`IEKF`的融合定位能够稳定运行
* 观测到的点云与地图中的点云重合度良好