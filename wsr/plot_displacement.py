#!/usr/bin/env python

import pickle
import matplotlib.pyplot as plt
from pathlib import Path
import numpy as np
from mpl_toolkits.mplot3d import axes3d
import base64
import argparse

def dist(p0, p1):
    return np.linalg.norm(p1-p0)

def plot_displacement(odometry_fpath: Path, name=None):
    if not name:
        name = f"{odometry_fpath} displacement"
    with odometry_fpath.open("rb") as fd:
        odometry = pickle.load(fd)
        # odometry = [[od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z] for od in odometry]
        # odometry = np.array(odometry)
        # odometry.reshape((-1, 3))
        # odometry = np.cumsum(odometry, axis=1)

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.plot(odometry[:, 0], odometry[:, 1], odometry[:, 2], marker="o")
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.set_zlabel("Z (meters)")
    ax.axis("equal")
    plt.title(name)
    print(dist(odometry[0,:], odometry[-1,:]))



if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("odemetry_pkls", type=Path, nargs="+")
    args = ap.parse_args()
    for od_pkl in args.odemetry_pkls:
        plot_displacement(od_pkl)
    plt.show()
