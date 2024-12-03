''' ranging utils '''
import time
from pathlib import Path

def get_rssi(interface="wlo1"):
    wireless_path = Path("/proc/net/wireless")
    with wireless_path.open("r") as fd:
        for line in fd:
            if line.strip().startswith(interface):
                _, status, link, level, noise, *_ = line.split()
                break
    return status, link, level, noise

def rssi_capture(fname, duration_s=120):
    savefile = Path(fname)
    data = list()
    iterations = duration_s * 2
    for ii in range(iterations):
        data.append((ii, time.time(), *get_rssi()))
        time.sleep(0.5)
    with savefile.open("w+") as fd:
        fd.write("ii,time,status,link,level,noise\n")
        for row in data:
            fd.write(",".join([str(ii) for ii in row]) + "\n")
