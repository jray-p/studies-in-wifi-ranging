#!/usr/bin/env python

import socket
import pickle
import struct
import time

import rospy
from std_msgs.msg import Byte, String, ByteMultiArray
import base64

from feitcsi_parse import parseFeitCSI

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
def publish():
    pub = rospy.Publisher("/wsr/csi", String, queue_size=10)
    rospy.init_node("feitcsi_bridge", anonymous=False)
    print("sending start...")
    sock.sendto(b"feitcsi --mode measureinject --frequency 2412 --channel-width 20 --format NOHT --inject-delay 10000", ("localhost", 8008))
    print("waiting for messages...")
    while not rospy.is_shutdown():
        raw_data, addr = sock.recvfrom(1024)
        data = None
        try:
            if not raw_data:
                continue
            data = parseFeitCSI(raw_data)
        except struct.error:
            pass
        # rospy.loginfo(data)
        if data is not None:
            print(f"macs={set([d['header']['source_mac_string'] for d in data])}")
            send_data = pickle.dumps({"data": data, "time":time.time()})
            pub.publish(base64.b64encode(send_data).decode("utf-8"))

if __name__ == "__main__":
    try:
        publish()
    finally:
        sock.sendto(b"stop", ("localhost", 8008))
