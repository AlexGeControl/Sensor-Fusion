#!/usr/bin/python

import os

import rospkg
import rospy

from gnss_ins_sim.sim import imu_model
from gnss_ins_sim.sim import ins_sim

from std_msgs.msg import String
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

def get_gnss_ins_sim(motion_def_file, fs_imu, fs_gps):
    '''
    Generate simulated GNSS/IMU data using specified trajectory.
    '''
    #### choose a built-in IMU model, typical for IMU381
    imu_err = 'mid-accuracy'
    # generate GPS and magnetometer data
    imu = imu_model.IMU(accuracy=imu_err, axis=9, gps=True)

    # init simulation:
    sim = ins_sim.Sim(
        [fs_imu, fs_gps, fs_imu],
        motion_def_file,
        ref_frame=1,
        imu=imu,
        mode=None,
        env=None,
        algorithm=None
    )
    
    # run:
    sim.run(1)

    # get simulated data:
    rospy.logwarn(
        "Simulated data size {}".format(
            len(sim.dmgr.get_data_all('gyro').data[0])
        )
    )
    for gyro, accel in zip(
        # a. gyro
        sim.dmgr.get_data_all('gyro').data[0], 
        # b. accel
        sim.dmgr.get_data_all('accel').data[0]
    ):
        yield {
            # a. gyro:
            'gyro_x': gyro[0],
            'gyro_y': gyro[1],
            'gyro_z': gyro[2],
            # b. accel:
            'accel_x': accel[0],
            'accel_y': accel[1],
            'accel_z': accel[2]
        }


def gnss_ins_sim_publisher():
    """
    Publish simulated GNSS/IMU data
    """
    
    # ensure gnss_ins_sim_node is unique:
    rospy.init_node('gnss_ins_sim_node')
    
    # TODO: move all constants to config:
    pub = rospy.Publisher('~sim/sensor/imu', Imu, queue_size=1000000)
    
    # generate simulated data:
    motion_def_path = os.path.join(
        rospkg.RosPack().get_path('gnss_ins_sim'), 'config', 'motion_def', 'motion_def-3d.csv'
    )
    imu_simulator = get_gnss_ins_sim(
        # motion def file:
        motion_def_path,
        # gyro-accel/gyro-accel-mag sample rate:
        100.0,
        # GPS sample rate:
        10.0
    )

    rate = rospy.Rate(100) # 100 Hz
    while not rospy.is_shutdown():
        # get measurement:
        try:
            measurement = next(imu_simulator)
        except StopIteration:
            break

        # init:
        msg = Imu()
        # a. set header:
        msg.header.frame_id = 'NED'
        msg.header.stamp = rospy.Time.now()
        # b. set orientation estimation:
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0
        # c. gyro:
        msg.linear_acceleration.x = measurement['gyro_x']
        msg.linear_acceleration.y = measurement['gyro_y']
        msg.linear_acceleration.z = measurement['gyro_z']
        msg.angular_velocity.x = measurement['accel_x']
        msg.angular_velocity.y = measurement['accel_y']
        msg.angular_velocity.z = measurement['accel_z']
        # finally:
        pub.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    try:
        gnss_ins_sim_publisher()
    except rospy.ROSInterruptException:
        pass