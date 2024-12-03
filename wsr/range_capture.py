#!/usr/bin/env python
import argparse
import logging
from pathlib import Path
import pandas as pd
import rospy
import base64
from nav_msgs.msg import Odometry
from std_msgs.msg import Byte, String, ByteMultiArray
import pickle
from wsr_pub import rssi_relative_dist
import signal
import numpy as np

LOG = logging.getLogger()

class RangeCapture:
    def __init__(self, mac, outfile):
        self.est_ranges, self.act_ranges = list(), list()
        self.rss1, self.rss2 = list(), list()
        self.locx, self.locy, self.locz = list(), list(), list()
        self.currx, self.curry, self.currz = 0, 0, 0
        self.mac = mac
        self.times = list()
        self.outfile = outfile
        rospy.init_node("range_capture", anonymous=False)

    def update_location(self, msg):
        self.currx = msg.pose.pose.position.x
        self.curry = msg.pose.pose.position.y
        self.currz = msg.pose.pose.position.z

    def handle_csi(self, msg):
        csi = pickle.loads(base64.b64decode(msg.data))
        csi_data = csi['data'][0]
        rx_time = csi['time']
        if csi_data['header']['source_mac_string'] != self.mac:
            return

        self.est_ranges.append(rssi_relative_dist(-1*csi_data['header']['rssi1'], r0=1, pr0=-34))
        self.act_ranges.append(np.linalg.norm(np.array([self.currx, self.curry, self.currz])))
        self.locx.append(self.currx)
        self.locy.append(self.curry)
        self.locz.append(self.currz)
        self.rss1.append(csi_data['header']['rssi1'])
        self.rss2.append(csi_data['header']['rssi2'])
        self.times.append(rx_time)

    def stop(self, *args):
        print("stopping...")
        df = pd.DataFrame({
            "x":self.locx,
            "y":self.locy,
            "z":self.locz,
            "time":self.times,
            "est_range":self.est_ranges,
            "act_range":self.act_ranges,
            "rssi1":self.rss1,
            "rssi2":self.rss2,
        })
        df.to_pickle(self.outfile)
        rospy.signal_shutdown("C-c called by user")

    def run(self):
        rospy.Subscriber("/lio_sam/mapping/odometry", Odometry, self.update_location)
        rospy.Subscriber("/wsr/csi", String, self.handle_csi)
        signal.signal(signal.SIGINT, self.stop)
        rospy.spin()



def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--verbose", "-v", action="store_true")
    ap.add_argument("--outfile", default=Path("range_data.gz"))
    ap.add_argument("--mac", default="00:16:ea:12:34:58")


    args = ap.parse_args()
    level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(level=level, format="%(asctime)s :: %(levelname)s :: %(name)s :: %(message)s")
    RangeCapture(args.mac, args.outfile).run()





if __name__ == '__main__':
    main()
