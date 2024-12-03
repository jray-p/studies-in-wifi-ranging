# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
import serial
import argparse
import struct

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self, topic, device):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, topic, 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.ser = serial.Serial(device)
        self.ser.baudrate = 115200

    def timer_callback(self):
        data = self.ser.read(13)
        # print(f"got '{data}' from serial interface")
        ss = struct.unpack("<IIbI", data)
        # print(ss)

        msg = String()
        msg.data = f"rtt_ns:{ss[0]},rtt_ns_raw:{ss[1]},rssi:{ss[2]},dist_est:{ss[3]/100}"
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="topic")
    ap.add_argument("--dev", default="/dev/ttyUSB1")

    args = ap.parse_args()
    minimal_publisher = MinimalPublisher(args.topic, args.dev)

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
