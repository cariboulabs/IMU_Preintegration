#! /usr/bin/env python3

import csv
import matplotlib.pyplot as plt
import argparse
import numpy as np
from dataclasses import dataclass

def quat_to_rot(q : np.ndarray) -> np.ndarray:
     #quat is in w,x,y,z format
    q = q / np.linalg.norm(q)
    w, x, y, z = q
    R = np.array([[1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
                  [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
                  [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]])
    return R

def plot_pose3_on_axes(axes : plt.Axes, t : np.ndarray, q : np.ndarray, axis_length=0.1) -> None:
    """
    Plot a 3D pose on given axis `axes` with given `axis_length`.
    Args:
        axes (matplotlib.axes.Axes): Matplotlib axes.
        point (gtsam.Point3): The point to be plotted.
        linespec (string): String representing formatting options for Matplotlib.
    """

    # get rotation and translation (center)
    gRp = quat_to_rot(q)
    origin = t

    # draw the camera axes
    x_axis = origin + gRp[:, 0] * axis_length
    line = np.append(origin[np.newaxis], x_axis[np.newaxis], axis=0)
    axes.plot(line[:, 0], line[:, 1], line[:, 2], 'r-')

    y_axis = origin + gRp[:, 1] * axis_length
    line = np.append(origin[np.newaxis], y_axis[np.newaxis], axis=0)
    axes.plot(line[:, 0], line[:, 1], line[:, 2], 'g-')

    z_axis = origin + gRp[:, 2] * axis_length
    line = np.append(origin[np.newaxis], z_axis[np.newaxis], axis=0)
    axes.plot(line[:, 0], line[:, 1], line[:, 2], 'b-')

def set_axes_equal(ax):
    limits = np.array([
        ax.get_xlim3d(),
        ax.get_ylim3d(),
        ax.get_zlim3d(),
    ])

    origin = np.mean(limits, axis=1)
    radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))

    ax.set_xlim3d([origin[0] - radius, origin[0] + radius])
    ax.set_ylim3d([origin[1] - radius, origin[1] + radius])
    ax.set_zlim3d([origin[2] - radius, origin[2] + radius])

@dataclass
class NavState:
    q: np.ndarray
    t: np.ndarray
    v : np.ndarray

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Plot trajectory')
    parser.add_argument('csv_file', type=str, help='Path to the file containing the trajectory')
    args = parser.parse_args()
    csv_file = args.csv_file

    traj_data : list[NavState] = []
    with open(csv_file, 'r') as f:
        reader = csv.reader(f)
        #get rid of the header
        next(reader)
        for row in reader:
            q = np.array([float(x) for x in row[:4]])
            t = np.array([float(x) for x in row[4:7]])
            v = np.array([float(x) for x in row[7:10]])
            state = NavState(q,t,v)
            traj_data.append(state)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for state in traj_data:
        plot_pose3_on_axes(ax, state.t, state.q)
    set_axes_equal(ax)
    plt.show()
