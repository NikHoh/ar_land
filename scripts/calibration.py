from argparse import ArgumentParser
from scipy.optimize import minimize

import numpy as np
import csv
import os


def _f_part(x, m):
    """
    Computes the "calibration error" for a single measurement:
    If the accelerometer is not moving the length of the vector of accelerations should be 9.81m/s².
    Here the squared error is calculated after appliying offset and scaling factors:
    """
    fx = (m[0] - x[0])*x[1]
    fy = (m[1] - x[2])*x[3]
    fz = (m[2] - x[4])*x[5]
    acc_abs = np.sqrt(fx**2 + fy**2 + fz**2)
    return (acc_abs - 9.81)**2 # squared error


def f(x, ms):
    """
    Computes the current "calibration error" for the given parameters/measurements.

    x: Calibration parameters of the form [x_bias, x_scale, y_bias, y_scale, z_bias, z_scale]
    ms: Accelerometer measurements to use for the calibration. Should have the form:
            [[ax_0, ay_0, az_0],
             [ax_1, ay_1,  ...], ...]
        Measurements should be made in all six orientations.
    """
    return sum([_f_part(x, m) for m in ms])


def main():
    parser = ArgumentParser(description="Accelerometer Calibration")
    parser.add_argument("-f", dest="filename", required=True,
                    help="Input csv file with two accelerometer measurements.", metavar="FILE",
                    type=str)
    args = parser.parse_args()

    # read measurements and check data
    with open(args.filename, 'r') as csvFile:
        reader = csv.reader(csvFile)
        measurements = list(reader)
    if not measurements or len(measurements) != 7:
        raise ValueError('Invalid input data ...')

    # remove csv head and convert to float
    measurements.pop(0) 
    measurements = [[float(mi) for mi in m] for m in measurements]

    # run optimization
    x0 = np.array([0., 1., 0., 1., 0., 1.])
    res = minimize(f, x0, measurements)

    print(res.message)
    print('Error=', res.fun)

    # write results to file
    out_dir = os.path.dirname(args.filename)
    out_path = os.path.join(out_dir, 'accel_calib.csv')
    with open(out_path, 'w') as out_file:
        out_file.write('x_bias,x_scale,y_bias,y_scale,z_bias,z_scale\n')
        out_file.write(','.join([str(xi) for xi in list(res.x)]))

if __name__ == "__main__":
    main()
