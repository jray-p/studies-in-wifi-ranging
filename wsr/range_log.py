#!/usr/bin/env python
import argparse
import logging
from pathlib import Path
import pandas as pd
import rospy
import base64
from nav_msgs.msg import Odometry
from std_msgs.msg import Byte, String, ByteMultiArray, Float64MultiArray
import pickle
import signal
import numpy as np
from collections import deque

LOG = logging.getLogger()

CALIBRATED_DIST_M = 1
# CALIBRATED_RSS_DB = -34 # los
CALIBRATED_RSS_DB = -41 # nlos

ROLLING_WINDOW = 50
TIMEOUT_S = 1

def rssi_relative_dist(rss, n=2, r0=CALIBRATED_DIST_M, pr0=CALIBRATED_RSS_DB):
    return r0 / (10 ** ((rss - pr0) / (10 * n)))

class RangeCapture:
    def __init__(self, mac):
        self.est_ranges = list()
        self.rss1, self.rss2 = list(), list()
        self.mac = mac
        self.pub = rospy.Publisher("/lio_sam/range", Float64MultiArray, queue_size=10)
        self._ranges = list()
        self.timeout_timer = None
        rospy.init_node("range_log", anonymous=False)

    def handle_timeout(self, _):
        if self._ranges:
            self.publish_ranges()

    def publish_ranges(self):
        if self.timeout_timer is not None:
            self.timeout_timer.shutdown()
            self.timeout_timer = None
        ranges = np.array(self._ranges)
        r_meas = np.median(ranges)
        r_std = np.std(ranges)
        self.pub.publish(data=[r_meas, r_std])
        self._ranges.clear()


    def handle_csi(self, msg):
        csi = pickle.loads(base64.b64decode(msg.data))
        csi_data = csi['data'][0]
        rx_time = csi['time']
        if csi_data['header']['source_mac_string'] != self.mac:
            return

        est_range = rssi_relative_dist(-1*csi_data['header']['rssi1'])
        print("est range (m):", est_range, "\trss1=", csi_data['header']['rssi1'])
        if not self._ranges:
            # start timer
            self.timeout_timer = rospy.Timer(rospy.Duration(TIMEOUT_S), self.handle_timeout, oneshot=True)
        self._ranges.append(est_range)
        if len(self._ranges) >= ROLLING_WINDOW:
            self.publish_ranges()
        # self.pub.publish()

    def run(self):
        # rospy.Subscriber("/lio_sam/mapping/odometry", Odometry, self.update_location)
        rospy.Subscriber("/wsr/csi", String, self.handle_csi)
        rospy.spin()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--verbose", "-v", action="store_true")
    ap.add_argument("--mac", default="00:16:ea:12:34:58")


    args = ap.parse_args()
    level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(level=level, format="%(asctime)s :: %(levelname)s :: %(name)s :: %(message)s")
    RangeCapture(args.mac).run()





if __name__ == '__main__':
    main()
