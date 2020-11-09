#!/usr/bin/python


import os
import sys

import argparse
from collections import namedtuple

import math
import numpy as np
import pandas as pd
import json


Config = namedtuple('Config', ['input'], verbose=False)


def get_arguments():
    """ 
    Get command-line arguments

    """
    # init parser:
    parser = argparse.ArgumentParser("Estimate IMU deterministic error using separated calibration.")

    # add required and optional groups:
    required = parser.add_argument_group('Required')
    optional = parser.add_argument_group('Optional')

    # add required:
    required.add_argument(
        "-i", dest="input", help="Input directory of IMU measurement",
        required=True, type=str
    )

    # parse arguments:
    return parser.parse_args()


def estimate_gyro_scale(data, axis):
    """
    Estimate scale of gyro measurement
    """
    # generate labels:
    stage_id_pos = 'rotate_{}_pos'.format(axis)
    stage_id_neg = 'rotate_{}_neg'.format(axis)
    input_id = 'ref_gyro_{}'.format(axis)

    # get measurements:
    measurement = (
        data.loc[
            stage_id_pos == data['stage'], 
            ['gyro_x', 'gyro_y', 'gyro_z', input_id]
        ].values - 
        data.loc[
            stage_id_neg == data['stage'], 
            ['gyro_x', 'gyro_y', 'gyro_z', input_id]
        ].values
    )
    
    N, _ = measurement.shape
    M = N / 2
    observation = measurement[M:, :3]
    control = measurement[M:, 3].mean()

    # build problem:
    b = observation.reshape((-1,))
    A = np.repeat(
        control*np.identity(3).reshape((1, 3, 3)),
        M,
        axis = 0
    ).reshape((-1, 3))

    # get scale:
    scale = np.dot(
        np.linalg.pinv(A), b
    ).reshape((3, 1))

    return scale


def estimate_gyro_bias(data, axis):
    """
    Estimate bias of gyro measurement
    """
    # generate labels:
    stage_id_pos = 'static_{}_pos'.format(axis)
    stage_id_neg = 'static_{}_neg'.format(axis)

    measurements_id = set('xyz')
    measurements_id.remove(axis)
    measurements_id = ['gyro_{}'.format(axis) for axis in measurements_id]

    # get measurements:
    measurement = (
        data.loc[
            stage_id_pos == data['stage'], 
            measurements_id
        ].values + 
        data.loc[
            stage_id_neg == data['stage'], 
            measurements_id
        ].values
    )

    return {
        k:v for k, v in zip(
            measurements_id, 
            0.5*measurement.mean(axis=0)*3600/math.pi*180
        )
    }


def get_accel_control(accel, axis):
    accel_x, accel_y, accel_z = accel

    if axis == 'x':
        return np.array([0.0, accel_y, 0.0])
    elif axis == 'y':
        return np.array([accel_x, 0.0, 0.0])

    return np.array([0.0, 0.0, accel_z])


def get_sub_A(accel):
    """
    Build A sub-matrix from accel control input
    """
    accel_x, accel_y, accel_z = accel

    return np.hstack(
        (
            np.array(
                [
                    [accel_x,     0.0,     0.0, accel_y, accel_z,     0.0,     0.0,     0.0,     0.0],
                    [    0.0, accel_y,     0.0,     0.0,     0.0, accel_x, accel_z,     0.0,     0.0],
                    [    0.0,     0.0, accel_z,     0.0,     0.0,     0.0,     0.0, accel_x, accel_y]
                ]
            ),
            np.identity(3)
        )
    )


def estimate_accel_params(data):
    """
    Estimate deterministic error params of accel measurement
    """
    # init problem:
    b = []
    A = []
    
    # generate labels:
    measurements_id = [
            'accel_x',     'accel_y',     'accel_z', 
        'ref_accel_x', 'ref_accel_y', 'ref_accel_z', 
    ]
    for axis in 'xyz':
        for stage_id in ['static_{}_{}'.format(axis, direction) for direction in ['neg', 'pos']]:
            measurement = data.loc[
                stage_id == data['stage'], 
                measurements_id
            ].values
            
            observation = measurement[:, :3]
            control = measurement[:, 3:].mean(axis=0)

            M, _ = measurement.shape
            sub_b = observation.reshape((-1,))
            sub_A = np.repeat(
                get_sub_A(
                    get_accel_control(control, axis) 
                ).reshape((1, 3, 12)),
                M,
                axis = 0
            ).reshape((-1, 12))
            
            b.append(sub_b)
            A.append(sub_A)
    
    # build problem:
    b = np.hstack(tuple(b))
    A = np.vstack(tuple(A))

    # get params
    params = np.dot(
        np.linalg.pinv(A), b
    )

    kx, ky, kz, sxy, sxz, syx, syz, szx, szy, epsilon_x, epsilon_y, epsilon_z = params

    return {
        'kx': kx,
        'ky': ky,
        'kz': kz,
        'sxy': sxy,
        'sxz': sxz,
        'syx': syx,
        'syz': syz,
        'szx': szx,
        'szy': szy,
        'epsilon_x': epsilon_x,
        'epsilon_y': epsilon_y,
        'epsilon_z': epsilon_z
    }


def main(config):
    """
    Estimate IMU deterministic error using separated calibration.
    
    """
    # read measurements:
    data_filename = os.path.join(config.input, 'data.csv')
    data = pd.read_csv(data_filename)

    # get gyro scale:
    gyro_scale = np.hstack(
        tuple(estimate_gyro_scale(data, axis) for axis in 'xyz')
    )

    # get gyro bias:
    gyro_bias = {
        'gyro_x': [],
        'gyro_y': [],
        'gyro_z': []
    }
    for axis in 'xyz':
        bias_ = estimate_gyro_bias(data, axis)
        for k in bias_:
            gyro_bias[k].append(bias_[k])
    for k in gyro_bias:
        gyro_bias[k] = np.asarray(gyro_bias[k]).mean()

    # get accel params:
    accel_params = estimate_accel_params(data)

    # write result as json:
    results = {
        'accel': {
            'scale': [
                accel_params[ 'kx'], accel_params['sxy'], accel_params['sxz'],
                accel_params['syx'], accel_params[ 'ky'], accel_params['syz'],
                accel_params['szx'], accel_params['szy'], accel_params[ 'kz'] 
            ],
            'bias': {
                'x': accel_params['epsilon_x'],
                'y': accel_params['epsilon_y'],
                'z': accel_params['epsilon_z']
            }
        },
        'gyro': {
            'scale': list(gyro_scale.reshape((-1, ))),
            'bias': {
                'x': gyro_bias['gyro_x'],
                'y': gyro_bias['gyro_y'],
                'z': gyro_bias['gyro_z']
            }
        }
    }
    with open(
        os.path.join(config.input, "separated-calibration-results.json"), 
        'w'
    ) as json_file:
        json.dump(results, json_file)

    sys.exit(os.EX_OK) 

if __name__ == '__main__':
    # parse arguments:
    arguments = get_arguments()

    # format as config instance:
    config = Config(
        input = arguments.input
    )

    # run:
    main(config)
