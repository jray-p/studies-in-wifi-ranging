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

from std_msgs.msg import String
import argparse
import time
import sys


class MinimalSubscriber(Node):

    def __init__(self, csv_out, max_meas = None):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.header_written = False
        self.csv_out_path = csv_out
        self.meas_count = 0
        self.max_meas = max_meas

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

        fields, values = zip(*[m.split(":") for m in msg.data.split(",")])
        fields = ("time", *fields)
        values = (str(time.time()), *values)


        with open(self.csv_out_path, "a") as fd:
            if not self.header_written:
                fd.write(','.join(fields) + "\n")
                self.header_written = True
            fd.write(",".join(values) + "\n")
        self.meas_count += 1
        if self.max_meas and self.meas_count >= self.max_meas:
            self.get_logger().info("reached max measurements, exiting")
            sys.exit(0)



def main(args=None):
    rclpy.init(args=args)
    ap = argparse.ArgumentParser()
    ap.add_argument("csv_out")
    ap.add_argument("--max", default=None, type=int)
    args = ap.parse_args()
    minimal_subscriber = MinimalSubscriber(args.csv_out, args.max)

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
