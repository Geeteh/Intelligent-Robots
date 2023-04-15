#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
import time
class Robot:
	def __init__(self):
		self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)							
		rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumped)
		self.state = 0
		
	def bumped(self, msg):
		if msg.bumper == 1 and self.state == 0:            
			self.state = 1
			self.timeSinceChange = time.time()
			
	def run(self):
		rate = rospy.Rate(10)
		twist = Twist()
		while not rospy.is_shutdown():
			if self.state == 0:
				twist.angular.z = 0
				twist.linear.x = 0.2
			elif self.state == 1:
				twist.linear.x = -0.2	
				self.t = time.time()		
				if self.t - self.timeSinceChange > 2:
					self.state = 2
					self.timeSinceChange = time.time()
			elif self.state == 2:
				twist.linear.x = 0
				twist.angular.z = 0.8
				self.t = time.time()		
				if self.t - self.timeSinceChange > 2:
					self.state = 0
					self.timeSinceChange = time.time()

			self.pub.publish(twist)
			rate.sleep()

rospy.init_node('prison_break')
robot = Robot()
robot.run()
	
