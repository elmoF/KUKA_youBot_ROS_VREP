#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
rospy.init_node('doubler')

def callback(msg):
	doubled = Int32()
	doubled.data = msg.data * 2
	pub.publish(doubled)
	print doubled

sub = rospy.Subscriber('number', Int32, callback)
pub = rospy.Publisher('doubled', Int32)
pub2 = rospy.Publisher('number', Int32)

test = Int32()
test.data = 2
pub2.publish(test)
rospy.spin()
