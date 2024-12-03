from utils import rssi_capture
from pathlib import Path
import matplotlib.pyplot as plt
import matplotlib

matplotlib.use("TkAgg")
import pandas as pd
from math import pi
from functools import partial
import numpy as np
from collections import defaultdict

c = 299792458
freq_hz = 2.412e9
wavelength_m = c / freq_hz
tx_power_dBm = 22

TEST_TYPES = ["norm", "nlos", "interf"]


def rss_relative_dist(rss, n=2, r0=1, pr0=-53):
    return r0 / (10 ** ((rss - pr0) / (10 * n)))


data = list()
for csv in Path(".").glob("*.csv"):
    print(csv)
    dist = int(csv.name[0])
    tag = csv.stem.split("_")[1]
    with csv.open("r") as fd:
        for ii, line in enumerate(fd):
            if ii == 0:
                continue
            ii, time, status, link, level, noise = line.strip().split(",")
            data.append(
                dict(
                    zip(
                        ["ii", "time", "status", "link", "level", "noise", "dist", "tag"],
                        (int(ii), float(time), status, float(link), float(level), float(noise), dist, tag),
                    )
                )
            )
df = pd.DataFrame(data)

dist_std = list()
pdata = defaultdict(dict)
for test_type in TEST_TYPES:
    pdata[test_type]["mean"] = list()
    pdata[test_type]["median"] = list()
dists = sorted(list(df["dist"].unique()))
for dd in dists:
    dist_std.append(df[df["dist"] == dd]["level"].std())
    for test_type in TEST_TYPES:
        pdata[test_type]["mean"].append(df[(df["dist"] == dd) & (df["tag"] == test_type)]["level"].mean())
        pdata[test_type]["median"].append(df[(df["dist"] == dd) & (df["tag"] == test_type)]["level"].median())


fig, ax = plt.subplots()
ax.set_title("RSSI over distance")
ax.set_xlabel("Distance (m)")
ax.set_ylabel("RSSI (dB)")
for test_type in TEST_TYPES:
    style = "-"
    if test_type == "nlos":
        style = "--"
    if test_type == "interf":
        style = "-."
    ax.plot(dists, pdata[test_type]["mean"], label=f"{test_type} mean", linestyle=style)
    ax.plot(dists, pdata[test_type]["median"], label=f"{test_type} median", linestyle=style)
fig.legend()
fig.savefig("RSSI_over_distance.png")

fig, ax = plt.subplots()
average_error = defaultdict(list)

ax.set_title("Median Error over Distance")
ax.set_xlabel("Distance (m)")
ax.set_ylabel("Median Distance Estimate Error (m)")
pr0 = pdata["norm"]["median"][0]
dist_func = partial(rss_relative_dist, pr0=pr0)

for dd in dists:
    for test_type in TEST_TYPES:
        average_error[test_type].append(
            np.abs(np.median(df[(df["dist"] == dd) & (df["tag"] == test_type)]["level"].map(dist_func).to_numpy() - dd))
        )
for test_type in TEST_TYPES:
    ax.plot(dists, average_error[test_type], label=test_type)
fig.legend()
fig.savefig("RSSI_Distance_median_error.png")
plt.show()
