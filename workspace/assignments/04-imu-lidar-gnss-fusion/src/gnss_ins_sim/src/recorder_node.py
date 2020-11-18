#!/usr/bin/python

import os

import rospkg
import rospy
import rosbag

import math
import numpy as np
import pandas as pd

from gnss_ins_sim.geoparams import geoparams
from gnss_ins_sim.attitude import attitude

from gnss_ins_sim.sim import imu_model
from gnss_ins_sim.sim import ins_sim

from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry


D2R = math.pi / 180.0
R2D = 180.0 / math.pi

def get_init_pose(stamp, motion_def_file):
    """
    Get init pose from motion def file
    """
    # parse:
    (
        lat, lon, alt,
        vx, vy, vz,
        yaw, pitch, roll
    ) = np.genfromtxt(
        motion_def_file, delimiter=',', skip_header=True, max_rows=1
    )

    # init:
    init_pose_msg = Odometry()

    # set header:
    init_pose_msg.header.stamp = stamp
    init_pose_msg.header.frame_id = '/map'
    init_pose_msg.child_frame_id = '/imu_link'

    # a. navigation frame, position:
    """
    (
        init_pose_msg.pose.pose.position.x, 
        init_pose_msg.pose.pose.position.y, 
        init_pose_msg.pose.pose.position.z
    ) = geoparams.lla2ecef(
        [
            lat*D2R, lon*D2R, alt
        ]
    )
    """
    (
        init_pose_msg.pose.pose.position.x, 
        init_pose_msg.pose.pose.position.y, 
        init_pose_msg.pose.pose.position.z
    ) = [
        0.0, 0.0, 0.0
    ]
    # b. navigation frame, orientation
    (
        init_pose_msg.pose.pose.orientation.w, 
        init_pose_msg.pose.pose.orientation.x, 
        init_pose_msg.pose.pose.orientation.y, 
        init_pose_msg.pose.pose.orientation.z
    ) = attitude.euler2quat(
        np.asarray(
            [yaw*D2R, pitch*D2R, roll*D2R]
        ),
        rot_seq='zyx'
    )

    # c. body frame, velocity:
    (
        init_pose_msg.twist.twist.linear.x, 
        init_pose_msg.twist.twist.linear.y, 
        init_pose_msg.twist.twist.linear.z
    ) = (
        vx, vy, vz
    )

    # finally:
    return init_pose_msg


def get_imu_msg(stamp, gyro, accel):
    """
    Get ROS Imu msg from simulation data
    """
    # init:
    imu_msg = Imu()

    # a. set header:
    imu_msg.header.stamp = stamp
    imu_msg.header.frame_id = '/imu_link'

    # b. set orientation estimation:
    imu_msg.orientation.w = 1.0
    imu_msg.orientation.x = 0.0
    imu_msg.orientation.y = 0.0
    imu_msg.orientation.z = 0.0

    # c. gyro:
    (
        imu_msg.angular_velocity.x,
        imu_msg.angular_velocity.y,
        imu_msg.angular_velocity.z
    ) = gyro

    # d. accel:
    (
        imu_msg.linear_acceleration.x,
        imu_msg.linear_acceleration.y,
        imu_msg.linear_acceleration.z 
    ) = accel
    
    # finally:
    return imu_msg 


def get_gps_pos_msg(stamp, gps_pos):
    """
    Get ROS NavSatFix msg from simulation data
    """
    # init:
    gps_pos_msg = NavSatFix()

    # set header:
    gps_pos_msg.header.stamp = stamp
    gps_pos_msg.header.frame_id = '/imu_link'

    # set LLA:
    (
        gps_pos_msg.latitude,
        gps_pos_msg.longitude,
        gps_pos_msg.altitude
    ) = (
        gps_pos[0] * R2D,
        gps_pos[1] * R2D,
        gps_pos[2]
    )

    return gps_pos_msg


def get_gps_vel_msg(stamp, gps_vel):
    """
    Get ROS TwistStamped msg from simulation data
    """
    # init:
    gps_vel_msg = TwistStamped()

    # set header:
    gps_vel_msg.header.stamp = stamp
    gps_vel_msg.header.frame_id = '/imu_link'

    # set NEU speed:
    (
        gps_vel_msg.twist.linear.x,
        gps_vel_msg.twist.linear.y,
        gps_vel_msg.twist.linear.z
    ) = gps_vel 

    return gps_vel_msg


def get_pose_msg(stamp, ref_pos, ref_vel, ref_att_quat):
    """
    Get ROS Odometry msg from simulation data
    """
    # init:
    pose_msg = Odometry()

    # set header:
    pose_msg.header.stamp = stamp
    pose_msg.header.frame_id = '/map'
    pose_msg.child_frame_id = '/map'

    # a. navigation frame, position:
    (
        pose_msg.pose.pose.position.x, 
        pose_msg.pose.pose.position.y, 
        pose_msg.pose.pose.position.z
    ) = (
        ref_pos[0] * R2D,
        ref_pos[1] * R2D,
        ref_pos[2]
    )
    
    # b. navigation frame, orientation
    (
        pose_msg.pose.pose.orientation.w, 
        pose_msg.pose.pose.orientation.x, 
        pose_msg.pose.pose.orientation.y, 
        pose_msg.pose.pose.orientation.z
    ) = ref_att_quat

    # c. body frame, velocity:
    (
        pose_msg.twist.twist.linear.x, 
        pose_msg.twist.twist.linear.y, 
        pose_msg.twist.twist.linear.z
    ) = ref_vel
    
    # finally:
    return pose_msg 


def get_gnss_ins_sim(motion_def_file, fs_imu, fs_gps):
    '''
    Generate simulated GNSS/IMU data using specified trajectory.
    '''
    #
    # set IMU model:
    #

    # for error-state Kalman filter observability & the degree of observability analysis
    # remove deterministic error and random noise:
    imu_err = {
        # 1. gyro:
        # a. random noise:
        # gyro angle random walk, deg/rt-hr
        'gyro_arw': np.array([0.00, 0.00, 0.00]),
        # gyro bias instability, deg/hr
        'gyro_b_stability': np.array([0.00, 0.0, 0.0]),
        # gyro bias isntability correlation time, sec
        # 'gyro_b_corr': np.array([100.0, 100.0, 100.0]),
        # b. deterministic error:
        'gyro_b': np.array([0.00, 0.00, 0.00]),
        'gyro_k': np.array([1.00, 1.00, 1.00]),
        'gyro_s': np.array([0.00, 0.00, 0.00, 0.00, 0.00, 0.00]),
        # 2. accel:
        # a. random noise:
        # accel velocity random walk, m/s/rt-hr
        'accel_vrw': np.array([0.00, 0.00, 0.00]),
        # accel bias instability, m/s2
        'accel_b_stability': np.array([0.00, 0.00, 0.00]),
        # accel bias isntability correlation time, sec
        # 'accel_b_corr': np.array([100.0, 100.0, 100.0]),
        # b. deterministic error:
        'accel_b': np.array([0.00, 0.00, 0.00]),
        'accel_k': np.array([1.00, 1.00, 1.00]),
        'accel_s': np.array([0.00, 0.00, 0.00, 0.00, 0.00, 0.00]),
        # 3. mag:
        'mag_si': np.eye(3) + np.random.randn(3, 3)*0.0, 
        'mag_hi': np.array([10.0, 10.0, 10.0])*0.0,
        'mag_std': np.array([0.1, 0.1, 0.1])
    }
    gps_err = {
        'stdp': np.array([0.0, 0.0, 0.0]),
        'stdv': np.array([0.0, 0.0, 0.0])
    }
    odo_err = {
        'scale': 1.00,
        'stdv': 0.0
    }
    # generate GPS and magnetometer data:
    imu = imu_model.IMU(
        accuracy=imu_err, 
        axis=9, 
        gps=True, gps_opt=gps_err,
        odo=True, odo_opt=odo_err
    )

    # init simulation:
    sim = ins_sim.Sim(
        # here sync GPS with other measurements as marker:
        [fs_imu, fs_imu, fs_imu],
        motion_def_file,
        # use NED frame:
        ref_frame=0,
        imu=imu,
        mode=None,
        env=None,
        algorithm=None
    )
    
    # run:
    sim.run(1)

    # get simulated data:
    rospy.logwarn(
        """
        GNSS-INS-Sim Summary:
        \tIMU Measurement:
        \t\tGyroscope: {}
        \t\tAccelerometer: {}
        \t\tMagnetometer: {}
        \tGNSS Measurement:
        \t\tLLA: {}
        \t\tNED Velocity: {}
        \tOdometry:
        \t\tVelocity: {}
        \tReference Trajectory:
        \t\tPosition: {}
        \t\tVelocity: {}
        \t\tOrientation in Quaternion: {}
        """.format(
            # a. IMU:
            repr(sim.dmgr.get_data_all('gyro').data[0].shape),
            repr(sim.dmgr.get_data_all('accel').data[0].shape),
            repr(sim.dmgr.get_data_all('mag').data[0].shape),
            # b. GNSS:
            repr(sim.dmgr.get_data_all('gps').data[0][:, :3].shape),
            repr(sim.dmgr.get_data_all('gps').data[0][:, 3:].shape),
            # c. odometry:
            repr(sim.dmgr.get_data_all('odo').data[0].shape),
            # d. reference trajectory:
            repr(sim.dmgr.get_data_all('ref_pos').data.shape),
            repr(sim.dmgr.get_data_all('ref_vel').data.shape),
            repr(sim.dmgr.get_data_all('ref_att_quat').data.shape),
        )
    )

    # init timer:
    timestamp_start = rospy.Time.now()
    STEP_SIZE = 1.0 / fs_imu

    # yield init pose:
    init_pose_msg = get_init_pose(timestamp_start, motion_def_file)
    yield init_pose_msg

    # yield measurements:
    for i, (
        # a. IMU:
        gyro, accel, mag,
        # b. GNSS:
        gps_pos, gps_vel,
        # c. odometry:
        odo,
        # d. reference trajectory:
        ref_pos, ref_vel, ref_att_quat
    ) in enumerate(
        zip(
            # a. IMU:
            sim.dmgr.get_data_all('gyro').data[0],
            sim.dmgr.get_data_all('accel').data[0],
            sim.dmgr.get_data_all('mag').data[0], 
            # b. GNSS:
            sim.dmgr.get_data_all('gps').data[0][:, :3],
            sim.dmgr.get_data_all('gps').data[0][:, 3:],
            # c. odometry velocity:
            sim.dmgr.get_data_all('odo').data[0],
            # d. reference trajectory:
            sim.dmgr.get_data_all('ref_pos').data,
            sim.dmgr.get_data_all('ref_vel').data,
            sim.dmgr.get_data_all('ref_att_quat').data
        )
    ):  
        # generate timestamp:
        stamp = timestamp_start + rospy.Duration.from_sec(i * STEP_SIZE) 

        # a. IMU:
        imu_msg = get_imu_msg(
            stamp, gyro, accel
        )

        # b. GNSS:
        gps_pos_msg = get_gps_pos_msg(
            stamp, gps_pos
        )
        gps_vel_msg = get_gps_vel_msg(
            stamp, gps_vel
        )

        # d. reference trajectory:
        reference_pose_msg = get_pose_msg(
            stamp, ref_pos, ref_vel, ref_att_quat
        )

        yield (imu_msg, gps_pos_msg, gps_vel_msg, reference_pose_msg)


def gnss_ins_sim_recorder():
    """
    Record simulated GNSS/IMU data as ROS bag
    """
    # ensure gnss_ins_sim_node is unique:
    rospy.init_node('gnss_ins_sim_recorder_node')

    # parse params:
    motion_def_name = rospy.get_param('/gnss_ins_sim_recorder_node/motion_file')

    sample_freq_imu = rospy.get_param('/gnss_ins_sim_recorder_node/sample_frequency/imu')
    sample_freq_gps = rospy.get_param('/gnss_ins_sim_recorder_node/sample_frequency/gps')

    topic_name_init_pose = rospy.get_param('/gnss_ins_sim_recorder_node/topic_name/init_pose')
    topic_name_imu = rospy.get_param('/gnss_ins_sim_recorder_node/topic_name/imu')
    topic_name_gps_pos = rospy.get_param('/gnss_ins_sim_recorder_node/topic_name/gps_pos')
    topic_name_gps_vel = rospy.get_param('/gnss_ins_sim_recorder_node/topic_name/gps_vel')
    topic_name_reference_trajectory = rospy.get_param('/gnss_ins_sim_recorder_node/topic_name/reference_trajectory')
    
    rosbag_output_path = rospy.get_param('/gnss_ins_sim_recorder_node/output_path')
    rosbag_output_name = rospy.get_param('/gnss_ins_sim_recorder_node/output_name')

    # generate simulated data:
    motion_def_path = os.path.join(
        rospkg.RosPack().get_path('gnss_ins_sim'), 'config', 'motion_def', motion_def_name
    )
    imu_simulator = get_gnss_ins_sim(
        # motion def file:
        motion_def_path,
        # gyro-accel/gyro-accel-mag sample rate:
        sample_freq_imu,
        # GPS sample rate:
        sample_freq_gps
    )

    with rosbag.Bag(
        os.path.join(rosbag_output_path, rosbag_output_name), 'w'
    ) as bag:
        # write init pose:
        init_pose_msg = next(imu_simulator)
        bag.write(topic_name_init_pose, init_pose_msg, init_pose_msg.header.stamp)

        # write measurements:
        for measurement in imu_simulator:
            # parse:
            (imu_msg, gps_pos_msg, gps_vel_msg, reference_pose_msg) = measurement

            # write:
            bag.write(topic_name_imu, imu_msg, imu_msg.header.stamp)
            bag.write(topic_name_gps_pos, gps_pos_msg, gps_pos_msg.header.stamp)
            bag.write(topic_name_gps_vel, gps_vel_msg, gps_vel_msg.header.stamp)
            bag.write(topic_name_reference_trajectory, reference_pose_msg, reference_pose_msg.header.stamp)


if __name__ == '__main__':
    try:
        gnss_ins_sim_recorder()
    except rospy.ROSInterruptException:
        pass