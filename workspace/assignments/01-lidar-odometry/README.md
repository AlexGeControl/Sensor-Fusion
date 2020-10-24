# Sensor Fusion: Lidar Odometry -- 多传感器融合定位: 激光里程计

This is the solution of Assignment 01 of Sensor Fusion from [深蓝学院](https://www.shenlanxueyuan.com/course/261).

深蓝学院从多传感器融合定位第1节Lidar Odometry答案. 版权归深蓝学院所有. 请勿抄袭.

---

## Problem Statement

---

### 1. 了解 KITTI 数据集,完成相关依赖库的安装,运行作业代码,提供程序运行成功的截图

Docker运行环境参见[here](https://github.com/AlexGeControl/Sensor-Fusion)

所选的KITTI Test Drive数据为`2011_09_26_drive_0005_sync`, 转换得到的ROS Bag回放时的RViz截图如下所示:

<img src="doc/images/01-kitti-to-bag-demo.png" width="100%" alt="KITTI Playback"/>

<img src="doc/images/01-test-frame-demo.png" width="100%" alt="Test Frame"/>

<img src="doc/images/01-front-end-demo.png" width="100%" alt="Front End"/>

---

### 2. 在作业代码中, 仿照写好的基于`NDT`的匹配, 重新建立一个基于`ICP`的里程计(可以使用`PCL`), 要求可在配置文件中切换两种方法. 之后使用`evo`评估两种算法得到的轨迹精度

---

### 3. 自己实现一个激光匹配的方法, 可以是`ICP`, `NDT`或者是`LOAM`. 新建一个接口类的实例,同样要求可在配置文件中对各方法进行切换. 最后和作业2中所用的开源的`ICP`, `NDT`进行精度比较.