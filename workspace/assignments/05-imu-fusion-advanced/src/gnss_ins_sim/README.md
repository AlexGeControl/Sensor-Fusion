# GNSS-INS-Sim for IMU Kalman Filter Fusion

ROS wrapper package for [gnss-ins-sim](https://github.com/Aceinna/gnss-ins-sim) data `ROS bag recording`, which includes:

---

# Data & Corresponding ROS Message

* IMU 
    
    * `Gyroscope and Accelerometer` as `ROS sensor_msgs::Imu`

    * `Magnetometer` as `ROS sensor_msgs::MagneticField`

* GNSS

    * `Latitude, Longitude and Altitude` as `ROS sensor_msgs::NavSatFix`

    * `Vehicle Speed in NED frame` as `ROS geometry_msgs::TwistStamped`

* Odometer

    * `Vehicle Speed (Only in X/Forward Direction) in Body Frame` as `ROS geometry_msgs::TwistStamped`

* Reference Trajectory

    * `Reference Pose (P, V, Orientation) Series` as `ROS nav_msgs::Odometry`

---

# Sample ROS Bag Output

```bash
$ rosbag info virtual_proving_ground.bag 

path:        virtual_proving_ground.bag
version:     2.0
duration:    1:19s (79s)
start:       Nov 21 2020 01:31:52.95 (1605922312.95)
end:         Nov 21 2020 01:33:12.94 (1605922392.94)
size:        13.5 MB
messages:    48001
compression: none [18/18 chunks]
types:       geometry_msgs/TwistStamped [98d34b0043a2093cf9d9345ab6eef12e]
             nav_msgs/Odometry          [cd5e73d190d741a2f92e81eda573aca7]
             sensor_msgs/Imu            [6a62c6daae103f4ff57a132d6f95cec2]
             sensor_msgs/MagneticField  [2f3b0b43eed0c9501de0fa3ff89a45aa]
             sensor_msgs/NavSatFix      [2d3a8cd499b9b4a0249fb98fd05cfa48]
topics:      /init_pose               1 msg     : nav_msgs/Odometry         
             /reference_pose       8000 msgs    : nav_msgs/Odometry         
             /sim/sensor/gps/fix   8000 msgs    : sensor_msgs/NavSatFix     
             /sim/sensor/gps/vel   8000 msgs    : geometry_msgs/TwistStamped
             /sim/sensor/imu       8000 msgs    : sensor_msgs/Imu           
             /sim/sensor/imu/mag   8000 msgs    : sensor_msgs/MagneticField 
             /sim/sensor/odo       8000 msgs    : geometry_msgs/TwistStamped
```