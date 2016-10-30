#!/usr/bin/env python
#Example 7-1 red!green light!
# Driving and stopping all 3 s

import rospy
from geometry_msgs.msg import Twist

# que_size tells rospy to only buffer a single outbound message
cmd_vel_pub = rospy.Publisher('cmd_vel',Twist, queue_size=1)
rospy.init_node('red_light_green_light')

red_light_twist =Twist() #constructor setzt alle Felder auf 0 - Stopp
green_light_twist =Twist()
green_light_twist.linear.x=0.9 #lineare Geschw. go straight ahead 0.5 meters per sec
green_light_twist.linear.y=0.6

red_light_twist = Twist()
red_light_twist.linear.x = -1.0
red_light_twist.linear.y= -0.6

driving_forward = False
light_change_time = rospy.Time.now()
rate = rospy.Rate(10)#HZ := 10 hz = message all 100 ms

while not rospy.is_shutdown():
    print 'wakeup'
    if driving_forward:
	# continously publish velocity command message, to not stop robot, by not receiving
	print 'forward'
	cmd_vel_pub.publish(green_light_twist) 
    else:
	print 'backward'
	cmd_vel_pub.publish(red_light_twist)
    if light_change_time < rospy.Time.now(): #check system time and toggle light
	print 'change'	
	driving_forward = not driving_forward
	light_change_time = rospy.Time.now() + rospy.Duration(3)
    print 'sleep'	
    rate.sleep() # cools down the cpu by not sending to much

