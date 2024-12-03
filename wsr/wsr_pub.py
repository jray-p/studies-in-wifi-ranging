#!/usr/bin/env python
import time
import argparse
import pickle

from std_msgs.msg import Byte, String, ByteMultiArray, Int32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import rospy
import base64
import numpy as np
from scipy.integrate import cumulative_trapezoid

from test_bartlett_profile import bartlett_profile


CALIBRATED_DIST_M = 1
CALIBRATED_RSS_DB = -34
CAPTURE_DURATION_S = 2
CAPTURE_RATE_S = 5
ODO_SRC = "liosam"

def rssi_relative_dist(rss, n=2, r0=CALIBRATED_DIST_M, pr0=CALIBRATED_RSS_DB):
    return r0 / (10 ** ((rss - pr0) / (10 * n)))

def displacement_from_accl(accl, dt=0.01, v0=0):
    ''' acc assumed to be 3,N where N is number of samples '''
    vel = cumulative_trapezoid(accl, dx=dt, initial=0) + v0
    return cumulative_trapezoid(vel, dx=dt, initial=0)

class WSRPublisher:
    def __init__(self, name, mac):
        self.name = name
        self.other_mac = mac
        self.curr_csi = None
        self.curr_imu = None # not sure this is the one
        self.pub = rospy.Publisher("/interbot_bearing", String, queue_size=10)
        rospy.init_node("wsr_pub", anonymous=False)
        self.start_time = None
        self.capture_duration_s = CAPTURE_DURATION_S
        self.csi = list()
        self.odometry = list()
        self.raw_imu = list()
        self.velocity = None

    def handle_csi(self, msg):
        # only append if actively capturing
        if not self.start_time is None and time.time() - self.start_time < self.capture_duration_s:
            self.csi.append(msg)

    def handle_wsr_trigger(self, msg):
        print("starting capture")
        self.start_time = time.time()
        self.csi.clear()

    def clear(self):
        ''' reset after processing capture '''
        self.csi.clear()
        self.raw_imu.clear()
        self.odometry.clear()
        self.start_time = None

    def update_velocity(self, msg):
        self.velocity = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])

    def handle_odometry(self, msg):
        if not self.start_time is None and time.time() - self.start_time < self.capture_duration_s:
            self.odometry.append(msg)
        elif not self.start_time is None: # we've timedout, need to end capture
            self.process()

    def handle_imu(self, msg):
        if not self.start_time is None and time.time() - self.start_time < self.capture_duration_s:
            self.raw_imu.append(msg)
        elif not self.start_time is None: # we've timedout, need to end capture
            self.process()

    def process(self):
        now = time.time()
        if ODO_SRC=="imu" and self.velocity is None:
            print("skipping because no initial velocity")
            self.clear()
            return

        try:
            csi = [pickle.loads(base64.b64decode(pack.data)) for pack in self.csi]
            csi_data = [d['data'][0] for d in csi]
            filtered_csi = [d for d in csi_data if d['header']['source_mac_string'] == self.other_mac]
            print(f"{len(filtered_csi)=} {len(csi_data)=}")

            if ODO_SRC == "imu":
                # convert imu to displacement and save that too
                acc = np.array([(ii.linear_acceleration.x, ii.linear_acceleration.y, ii.linear_acceleration.z) for ii in self.raw_imu])
                # accel is a N,3 so transpose to 3,N and then back
                self.velocity = self.velocity.reshape((-1, 1))
                # print("vel=", self.velocity)
                od_data = displacement_from_accl(acc.T, v0=self.velocity).T
                # print(od_data)
            else:
                od_data = np.array([[od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z] for od in self.odometry])
                print(od_data)
                od_data.reshape((-1, 3))

            az_deg, el_deg, _ = bartlett_profile(csi_data, od_data, self.other_mac)
            median_rss = np.median([-1 * c['header']['rssi1'] for c in filtered_csi])
            range_m = rssi_relative_dist(median_rss, r0=CALIBRATED_DIST_M, pr0=CALIBRATED_RSS_DB)
            self.pub.publish(f"az_deg={az_deg}, el_deg={el_deg}, range_m={range_m}")

            with open(f"{self.name}_imu_{now}.pkl", "wb") as fd:
                pickle.dump(self.raw_imu, fd)
            with open(f"{self.name}_csi_{now}.pkl", "wb") as fd:
                pickle.dump(self.csi, fd)
            with open(f"{self.name}_odometry_{now}.pkl", "wb") as fd:
                pickle.dump(od_data, fd)
            print(f"capture complete. got {len(self.csi)} csi packets and {len(self.odometry)} odometry packets.")

        except Exception as err:
            print(f"error:{err}")

        self.clear()

    def run(self):
        if ODO_SRC == "imu":
            rospy.Subscriber("/odometry/imu", Odometry, self.update_velocity)
            rospy.Subscriber("/imu/data", Imu, self.handle_imu)
        else:
            rospy.Subscriber("/lio_sam/mapping/odometry", Odometry, self.handle_odometry)

        rospy.Subscriber("/wsr/csi", String, self.handle_csi) # i should define a type maybe?
        rospy.Timer(rospy.Duration(CAPTURE_RATE_S), self.handle_wsr_trigger)
        rospy.spin()



if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument("name")
    ap.add_argument("mac")
    args = ap.parse_args()
    WSRPublisher(args.name, args.mac).run()
