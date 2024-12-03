#!/usr/bin/env python3
import pickle
import matplotlib.pyplot as plt
from pathlib import Path
import numpy as np

import base64
import argparse
from csi_file import parse_csi_file

def minmax(x):
    return np.min(x), np.max(x)

def plot_phase_per_mac(data, mac, chan, ax=None):
    filtered = [d for d in data if d['header']['source_mac_string'] == mac]
    csi_mat = np.array([msg['csi_matrix'] for msg in filtered]).squeeze(axis=3)
    if ax:
        ax.plot(np.angle(csi_mat[:, chan, 0]), label=mac)
    else:
        plt.plot(np.angle(csi_mat[:, chan, 0]), label=mac)

def plot_phase_macs(data, macs):
    fig, axs = plt.subplots(len(macs), sharex=True)
    for ii,mac in enumerate(macs):
        plot_phase_per_mac(data, mac, 0, axs[ii])
    plt.show()

def plot_channel_data(fpath:Path, subcarrier_chan:int, rx_chan:int, mac:str = None, name=None):
    if not name:
        name = f"{fpath} phase"
    # with fpath.open("rb") as fd:
    #     if b64:
    #         # raw_data = fd.read()
    #         raw_data = pickle.load(fd)
    #         data = [pickle.loads(base64.b64decode(pkl.data)) for pkl in raw_data]
    #         # data = raw_data
    #         data = [d['data'][0] for d in data]
    #     else:
    #         data = pickle.load(fd)

    # # lamp somehow got 2 tx at one point?
    # data = [d for d in data if d['csi_matrix'].shape == (num_sub, 2, 1)]

    # print(f"found macs in data: {macs}")
    # if mac is not None:
    #     data = [d for d in data if d['header']['source_mac_string'] == mac]

    # should be N messages x M subcarriers x R receivers x T tx?
    # only 1 tx, so just squeeze it to a NxMxR matrix

    # csi_mat = np.array([msg['csi_matrix'] for msg in data]).squeeze(axis=3)
    # csi_mat = np.array([msg['csi_matrix'] for msg in data])
    csi_mat, header = parse_csi_file(fpath, mac)
    macs = list(set(header['mac']))
    print(f"found macs in data: {macs}")

    print(csi_mat.shape)
    times = header['times']
    colors = [macs.index(mm) for mm in header["mac"]]
    rss1 = [-1 * rr for rr in header["rssi1"]]
    rss2 = [-1 * rr for rr in header["rssi2"]]
    rss3 = [-1 * rr for rr in header["rssi3"]]

    # plt.figure()
    fig, axs = plt.subplots(2, 1, sharex=True)
    # axs[0].scatter(times, np.unwrap(np.angle(csi_mat[:, subcarrier_chan, rx_chan] * csi_mat[:, subcarrier_chan, 1] )), s=10, marker="x", c=colors)
    axs[0].scatter(times, np.unwrap(np.angle(csi_mat[:, subcarrier_chan, 0])), s=10, marker="x", c="blue")
    # axs[0].scatter(times, np.angle(csi_mat[:, subcarrier_chan, 1]), s=10, marker="o", c="red")
    # axs[0].scatter(times, np.angle(csi_mat[:, subcarrier_chan, 2]), s=10, marker="*", c="green")
    # axs[0].scatter(times, np.angle(csi_mat[:, subcarrier_chan, rx_chan]), s=10, marker="x", c=colors)
    plt.xlabel("Time")
    axs[0].set_ylabel("Phase (radians)")

    axs[1].scatter(times, rss1, label="rssi1", c="blue", marker="x")
    # axs[1].scatter(times, rss2, label="rssi2", c="red", marker="o")
    # axs[1].scatter(times, rss3, label="rssi3", c="green", marker="*")
    axs[1].set_ylabel("Signal Strength (dB)")

    print(f"mean rssi1: {np.median(rss1)}")

    plt.title(name)

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("csi_pkls", type=Path, nargs="+")
    ap.add_argument("--subcarrier_chan", type=int, default=0)
    ap.add_argument("--rx_chan", type=int, default=0)
    ap.add_argument("--mac", default=None)
    args = ap.parse_args()

    for pkl in args.csi_pkls:
        plot_channel_data(pkl, args.subcarrier_chan, args.rx_chan, args.mac)
    plt.show()


# python3 ./plot_channel_data.py bonn.pkl --sub 28 --mac 00:16:ea:12:34
# python3 ./plot_channel_data.py lamp.pkl --sub 28 --mac 8c:f1:12:a5:45:d3
