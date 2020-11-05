#!/usr/bin/python


import os
import sys

import argparse
from collections import namedtuple

import numpy as np
import pandas as pd
import json
import matplotlib.pyplot as plt


Config = namedtuple('Config', ['input'], verbose=False)


def get_arguments():
    """ 
    Get command-line arguments

    """
    # init parser:
    parser = argparse.ArgumentParser("Visualize Allan Variance analysis results.")

    # add required and optional groups:
    required = parser.add_argument_group('Required')
    optional = parser.add_argument_group('Optional')

    # add required:
    required.add_argument(
        "-i", dest="input", help="Input directory of calibration results",
        required=True, type=str
    )

    # parse arguments:
    return parser.parse_args()


def main(config):
    """
    Enhance the original map with semantic info
    
    """
    # load calibration results:
    results = {
        "gyro": {
            "measurement_noise_stddev": 0.0,
            "bias_random_walk_stddev": 0.0
        },
        "acc": {
            "measurement_noise_stddev": 0.0,
            "bias_random_walk_stddev": 0.0
        }        
    }
    with open(
        os.path.join(config.input, "imu_calibration_results.json")
    ) as f:
        calib_results = json.load(f)

    results["gyro"]["measurement_noise_stddev"] = np.sqrt(
        (
            float(calib_results["measurements"]["gyro_x"]["N"])**2 + 
            float(calib_results["measurements"]["gyro_y"]["N"])**2 + 
            float(calib_results["measurements"]["gyro_z"]["N"])**2
        ) / 3
    )
    results["gyro"]["bias_random_walk_stddev"] = np.sqrt(
        (
            float(calib_results["measurements"]["gyro_x"]["K"])**2 + 
            float(calib_results["measurements"]["gyro_y"]["K"])**2 + 
            float(calib_results["measurements"]["gyro_z"]["K"])**2
        ) / 3
    )
    results["acc"]["measurement_noise_stddev"] = np.sqrt(
        (
            float(calib_results["measurements"]["acc_x"]["N"])**2 + 
            float(calib_results["measurements"]["acc_y"]["N"])**2 + 
            float(calib_results["measurements"]["acc_z"]["N"])**2
        ) / 3
    )
    results["acc"]["bias_random_walk_stddev"] = np.sqrt(
        (
            float(calib_results["measurements"]["acc_x"]["K"])**2 + 
            float(calib_results["measurements"]["acc_y"]["K"])**2 + 
            float(calib_results["measurements"]["acc_z"]["K"])**2
        ) / 3
    )

    with open(
        os.path.join(config.input, "allan-variance-analysis--summary.json"), 
        'w'
    ) as json_file:
        json.dump(results, json_file)

    # prepare canvas for Allan Variance curve:
    df_allan_variance_curve = pd.read_csv(
        os.path.join(config.input, "imu_calibration_results.csv")
    )

    # average over all axis:
    # a. gyro:
    df_allan_variance_curve["gyro_avg"] = (
        df_allan_variance_curve["gyro_x"]+df_allan_variance_curve["gyro_y"]+df_allan_variance_curve["gyro_z"]
    ) / 3
    # b. acc:
    df_allan_variance_curve["acc_avg"] = (
        df_allan_variance_curve["acc_x"]+df_allan_variance_curve["acc_y"]+df_allan_variance_curve["acc_z"]
    ) / 3

    plt.figure(figsize=(16, 9), dpi=70)    
    plt.loglog(df_allan_variance_curve["T"], np.sqrt(df_allan_variance_curve["gyro_avg"]))
    plt.grid(True)
    plt.title('Allan Variance Curve, Gyroscope, GNSS-INS-SIM Low Accurary IMU')
    plt.savefig(
        os.path.join(config.input, "allan-variance-curve--gyro.png")
    )

    plt.figure(figsize=(16, 9), dpi=70)    
    plt.loglog(df_allan_variance_curve["T"], np.sqrt(df_allan_variance_curve["acc_avg"]))
    plt.grid(True)
    plt.title('Allan Variance Curve, Accelemeter, GNSS-INS-SIM Low Accurary IMU')
    plt.savefig(
        os.path.join(config.input, "allan-variance-curve--acc.png")
    )

    sys.exit(os.EX_OK) 

if __name__ == '__main__':
    # parse arguments:
    arguments = get_arguments()

    config = Config(
        input = arguments.input
    )

    main(config)
