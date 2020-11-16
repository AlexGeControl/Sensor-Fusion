# 作业讲解

---

## 1. 仿真IMU数据, 并进行Allan方差分析

Allan Variance Analysis本质是以采样频率为自变量, 以测量值方差为因变量的回归分析. 模型的详细推导请参考压缩包中的文档MEMS-IMU-Comparison-using-Allan-Variance.pdf. 要注意的时, Allan Variance Analysis要求数据采集时, IMU需保持静止. 工程实现时, 将仿真数据生成与方差分析分别实现为一个ROS Package: 前者将GNSS-INS-SIM产生的数据直接写成ROS Bag, 后者进行方差分析, 并将所得Allan Variance曲线通过Python可视化&以JSON格式存储至硬盘. 代码实现参考GitHub链接

---

## 2. 设计一种转台旋转方案, 进行内参求解

标定过程完全按照任大佬的讲义进行即可. 此处重点关注对应的工程实现. 首先需要修改GNSS-INS-Sim的源码, 支持三个系统误差参数的配置, 修改后的源代码参见GitHub链接, 或者直接获取已发布在阿里云Registry的Docker镜像. 接着要把任大佬讲义的标定过程转化为GNSS-INS-Sim的motion_def. 此处交替使用Type 1与Type 2类型指令, 同时使用gps_visibility指示标定子阶段的切换, 简化后续的数据解析. 实现方法参见GitHub链接

---

## 3. Accel标定的LM推导与下三角模型的实现

首先要注意的是, LM方法的推导, 仅对Accelerometer进行. Gyroscope的变换过于复杂, 若坚持获取解析表达式的话, 建议使用SymPy. LM的推导重在问题构建, 该环节深蓝机器人学中的状态估计课程的参考教材State Estimation for Robotics已有非常好的描述, 若有疑惑, 请参考书中相关章节. 代码的工程修改非常简单, 请参考GitHub README进行.

---

## 4. 导航解算验证

导航解算严格按照任大佬的讲义进行即可. 整体流程的C++实现参考GitHub README. 工程实现上, 可以基于GitHub中imu_integration ROS Package进行. 此处借鉴深蓝VIO课程, 实现了ROS环境下基于轨迹解析表达式的IMU仿真, 该仿真相比GNSS-INS-Sim使用的Motion File, 定义运动有更好的灵活性, 同时已实现和ROS & RViz的集成. 可以通过替换estimator中的航迹解算函数, 轻松对比不同解算方法的性能. 具体实现细节参考GitHub.

---

## Wrap-Up

Keep Learning & Keep Coding!

