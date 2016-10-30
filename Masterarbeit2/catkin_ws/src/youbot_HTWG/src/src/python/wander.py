#!/usr/bin/env python
#Example 7-3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
	global g_range_ahend #to store min range of laser in front
	g_range_ahead = min(msg.ranges)

g_range_ahead = 1 # anything to start
scan_sub = rospy.Subscriber('scan_front',LaserScan,scan_callback)
# que_size tells rospy to only buffer a single outbound message
cmd_vel_pub = rospy.Publisher('cmd_vel',Twist, queue_size=1)
rospy.init_node ('wander')
state_change_time = rospy.Time.now()
driving_forward = True
rate = rospy.Rate(10) #HZ

while not rospy.is_shutdown():
	if driving_forward:# solange kein Objekt <0.8 oder time out
	    if (g_range_ahead < 0.8 or rospy.Time.now() < state_change_time):
			driving_forward = False
			state_change_time = rospy.Time.now() +rospy.Duration(5)
	else: # we're not driving_forward- spinning fuer 5s
		if rospy.Time.now() < state_change_time:
			driving_forward = False	#we're done spinning, time to go forward!
			state_change_time = rospe.Time.now()+ rospy.Duration(30)
	twist = Twist()
	if driving_forward:
		twist.linear.y = 1
	else:
		twist.angular.z = 1
	cmd_vel_pub.publish(twist)
	
	rate.sleep()

