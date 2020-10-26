# Sensor Fusion: Lidar Mapping -- 多传感器融合定位: 基于地图的定位

This is the solution of Assignment 02 of Sensor Fusion from [深蓝学院](https://www.shenlanxueyuan.com/course/261).

深蓝学院从多传感器融合定位第2节Lidar Mapping答案. 版权归深蓝学院所有. 请勿抄袭.

---

## Problem Statement

---

### 1. 闭环修正及精度评价

提供的工程框架中, 已经给出了闭环的流程和实现, 但是是基于ICP的, 这是在已知粗略位姿情况下的实现方式. 在未知粗略位姿情况下, 闭环检测的实现难度会加大. 要求使用前面讲过的`Scan Context`, 实现此功能, 并和已有的图优化功能完成对接, 实现`Loop Closure`修正. 并最终给出修正前后的轨迹精度对比. 

---

### 2. 位姿初始化

提供的工程框架中, 已经给出了位姿初始化功能, 但是是在起始位置的, 并且是基于已知粗略位姿的. 要求实现`地图中任意位置的位姿初始化`, 可以从三种难度等级中任选一种, 难度越高, 得分越高.