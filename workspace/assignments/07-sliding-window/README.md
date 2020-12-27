# Sensor Fusion: Sliding Window for Real-Time Lidar Localization -- 多传感器融合定位: 基于滑动窗口的实时定位

This is the solution of Assignment 07 of Sensor Fusion from [深蓝学院](https://www.shenlanxueyuan.com/course/261).

深蓝学院从多传感器融合定位第7节Sliding Window for Real-Time Localization答案. 版权归深蓝学院所有. 请勿抄袭.

---

## Problem Statement

---

## 1. 推导使用LOAM构建残差时, 与之相关联的两个位姿的Jacobian

### ANS

完整的推导过程参见[here](doc/derivation/01-loam-jacobians.pdf). 此处仅将结论摘录如下, 详细的符号定义参考推导文档.

<img src="doc/images/01-loam-jacobians.png" alt="Jacobians for LOAM Residual" width="100%" />

---

## 2. 实现基于Sliding-Window的实时定位(Will be Available on 03/01/2021)

### ANS 

算法中的关键计算参考[here](doc/derivation/02-sliding-window-for-real-time-lidar-localization.pdf)