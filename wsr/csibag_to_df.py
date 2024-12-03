#!/usr/bin/python
''' save csi data from bag playback to pickle '''

import argparse
import logging
from pathlib import Path
import pandas as pd
import rospy
import base64
from std_msgs.msg import Byte, String, ByteMultiArray

import pickle
import signal
import numpy as np

LOG = logging.getLogger()

class CSICapture:
    def __init__(self, outfile):
        self.outfile = outfile
        self.csi = list()
        rospy.init_node("csi_capture", anonymous=False)

    def handle_csi(self, msg):
        self.csi.append(pickle.loads(base64.b64decode(msg.data)))

    def stop(self, *args):
        print("stopping...")
        with open(self.outfile, "wb") as fd:
            pickle.dump(self.csi, fd)
        rospy.signal_shutdown("C-c called by user")

    def run(self):
        rospy.Subscriber("/wsr/csi", String, self.handle_csi)
        signal.signal(signal.SIGINT, self.stop)
        rospy.spin()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--verbose", "-v", action="store_true")
    ap.add_argument("--outfile", default=Path("range_data.gz"))


    args = ap.parse_args()
    level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(level=level, format="%(asctime)s :: %(levelname)s :: %(name)s :: %(message)s")
    CSICapture(args.outfile).run()

if __name__ == '__main__':
    main()
