import rospy
from std_msgs.msg import String
import time

rospy.init_node("wsr_trigger", anonymous=False)
pub = rospy.Publisher("/wsr/trigger", String, queue_size=10)
while True:
    time.sleep(2)
    pub.publish("trigger")
    print("triggered, sleeping...")
