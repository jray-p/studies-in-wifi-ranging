#!/usr/bin/env python

import argparse
import logging
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from wsr_pub import rssi_relative_dist
from kalman import KalmanFilter
LOG = logging.getLogger()


def plot_range_est(infile: Path, re_est_dist=False, title=None, z_correct=None, roll=None):
    df = pd.read_pickle(infile)
    fig, axs = plt.subplots(2, 2,constrained_layout=True, figsize=(11,7))
    est_range = df['est_range']

    # try different filters here
    if roll:
        df['rssi1'] = df.sort_values("time")['rssi1'].rolling(roll).median()

    if re_est_dist:
        est_range = rssi_relative_dist(-1*df['rssi1'], r0=4, pr0=-55)
        # est_range = rssi_relative_dist(-1*df['rssi1'], r0=1, pr0=-34)
    if z_correct:
        df['z'] = df['z'] + z_correct
        df['act_range'] = np.linalg.norm(np.vstack([df['x'],df['y'],df['z']]), axis=0)



    err = np.abs(est_range - df['act_range'])
    df['err'] = np.abs(est_range - df['act_range'])
    rel_err = est_range - df['act_range']
    df['rel_err'] = est_range - df['err']

    print("mean abs error 0-2 meters:", df[df['act_range'] < 2]['err'].mean(), df[df['act_range'] < 2]['err'].std())
    print("mean abs error 2-5 meters:", df[(df['act_range'] > 2) & (df['act_range'] <5)]['err'].mean(), df[(df['act_range'] > 2) & (df['act_range'] <5)]['err'].std())
    print("mean abs error 5-10 meters:", df[(df['act_range'] > 5) & (df['act_range'] <10)]['err'].mean(), df[(df['act_range'] > 5) & (df['act_range'] <10)]['err'].std())

    dt = df['time'].diff().mean()
    print(f"{dt=}")
    # F = np.array([[1, dt], [0, 1]])
    # H = np.array([0, 1]).reshape(1,2)
    # kalman_filter = KalmanFilter(F, H)

    axs[0][0].plot(df['time'], err)
    axs[0][0].set_xlabel("Time")
    axs[0][0].set_ylabel("Absolute Range Error (m)")
    axs[0][0].set_title("Range Error over Time")

    axs[1][0].hist(est_range - df['act_range'], bins=20)
    axs[1][0].set_xlabel("Range Error (m)")
    axs[1][0].set_ylabel("Number")
    axs[1][0].set_title("Range Error Histogram")


    axs[0][1].plot(df['x'], df['y'], marker="o")
    axs[0][1].set_xlabel("Location X-axis (m)")
    axs[0][1].set_ylabel("Location y-axis (m)")
    axs[0][1].set_title("Path")


    df.groupby("act_range").plot(x="act_range", y="err", kind="scatter", ax=axs[1][1])
    df.groupby("act_range")["err"].median().plot(color="red", ax=axs[1][1])
    axs[1][1].set_xlabel("Distance (m)")
    axs[1][1].set_ylabel("Absolute Range error")
    axs[1][1].set_title("Range Error over Distance")

    title = title or f"Range data {infile.as_posix()} (roll={roll})"
    fig.suptitle(title)
    plt.savefig(infile.with_suffix(".png"))
    plt.figure()
    plt.plot(df['rssi1'])
    plt.plot(df['rssi2'])
    plt.show()

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--verbose", "-v", action="store_true")
    ap.add_argument("--re-est-dist", action="store_true")
    ap.add_argument("--z-correct", default=None, type=int)
    ap.add_argument("--roll", default=None, type=int)
    ap.add_argument("infile", type=Path)
    ap.add_argument("--title", default=None)

    args = ap.parse_args()
    level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(level=level, format="%(asctime)s :: %(levelname)s :: %(name)s :: %(message)s")
    plot_range_est(args.infile, args.re_est_dist, args.title, args.z_correct, args.roll)

if __name__ == '__main__':
    main()
