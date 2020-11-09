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


def main(config):
    """
    Estimate IMU deterministic error using separated calibration.
    
    """
    # read measurements:
    data_gyro_filename = os.path.join(config.input, 'data_gyro.csv')
    data_gyro = pd.read_csv(data_gyro_filename)

    # get gyro scale:
    gyro_scale = np.hstack(
        tuple(estimate_gyro_scale(data_gyro, axis) for axis in 'xyz')
    )

    # get gyro bias:
    gyro_bias = {
        'gyro_x': [],
        'gyro_y': [],
        'gyro_z': []
    }
    for axis in 'xyz':
        bias_ = estimate_gyro_bias(data_gyro, axis)
        for k in bias_:
            gyro_bias[k].append(bias_[k])
    for k in gyro_bias:
        gyro_bias[k] = np.asarray(gyro_bias[k]).mean()

    # write result as json:
    results = {
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
