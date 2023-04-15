#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from beginner_tutorials.msg import BallLocation
import time
import numpy as np
from math import atan2, sin, cos, pi, sqrt
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class PID:
    def __init__(self, goal, kp, ki, kd, max):
        self.goal = goal
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max = max
        self.previous_error = 0
        self.integral = 0

    def get_output(self, measurement):
        error = self.goal - measurement
        self.integral = self.integral + error
        derivative = error - self.previous_error
        output = self.kp*error + self.ki*self.integral + self.kd*derivative
        output = max(output, -self.max)
        output = min(output, self.max)
        self.previous_error = error
        return output


class Puppy:
	def __init__(self):
		self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)					
		rospy.Subscriber('/ball_detector/ball_location', BallLocation, self.handle_ball_location)
		rospy.Subscriber('/odom', Odometry, self.handle_pose)
		self.bearing_pid = PID(320,2/320,0,0,0.667)
		self.distance_pid = PID(1.5,-0.5,0,0,0.25)
		self.theta_pid = PID(0, -1, 0, 0, 0.5)
		self.path_distance_pid = PID(0, -0.25, 0, 0, 0.25)
		self.x = 0
		self.y = 0
		self.theta = 0
		self.goal_x = 0
		self.goal_y = 0
		self.intermediate_goal_x = 0
		self.intermediate_goal_y = 0
		self.bearing = None
		self.distance = None
		self.distance_interim = 0
		self.bearing_interim = 0
		self.distance_final = 0
		self.bearing_final = 0
		self.state = "Search"
		
	def handle_ball_location(self, msg):
		if msg.bearing == -1 or msg.distance == -1:
			self.bearing = None
			self.distance = None
		else:
			self.bearing = msg.bearing
			self.distance = msg.distance
			
	def handle_pose(self, msg):
        	self.x = msg.pose.pose.position.x
        	self.y = msg.pose.pose.position.y
        	q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        	(_, _, self.theta) = euler_from_quaternion(q)
        	self.theta = ((self.theta + pi) % 2*pi) - pi

	def get_vector(self, from_x, from_y, to_x, to_y):
		bearing = atan2(to_y - from_y, to_x - from_x)
		distance = sqrt((from_x - to_x)**2 + (from_y - to_y)**2)
		return (bearing, distance)
			
	def run(self):
		rate = rospy.Rate(10)
		twist = Twist()
		while not rospy.is_shutdown():
			if self.state == "Search":
				twist.linear.x = 0
				twist.angular.z = 0.6
				if self.bearing is not None and self.distance is not None:
					self.state = "Approach"
					
					
			elif self.state == "Approach":
				twist.angular.z = 0
				if self.bearing is None or self.distance is None:
					self.state = "Search"
				else:
					error_bearing = self.bearing_pid.get_output(self.bearing)
					error_distance = self.distance_pid.get_output(self.distance)

					twist.angular.z = error_bearing
					twist.linear.x = error_distance

					#print(str(error_bearing), str(error_distance))
					
					if abs(error_distance) < 0.1:
						self.intermediate_goal_x = self.x + 1.5*cos(self.theta + pi/4)
						self.intermediate_goal_y = self.y + 1.5*sin(self.theta + pi/4)
						print("Current (x,y) = (", self.x, ",", self.y, ")")
						print("Intermediate Goal Calculated: (", self.intermediate_goal_x, self.intermediate_goal_y, ")")
						#self.x + 1.5*(sqrt(2)/2)
						self.goal_x = self.x + 3*cos(self.theta)
						self.goal_y = self.y + sin(self.theta)
						print("Final Goal Calculated: (", self.goal_x, self.goal_y, ")")
						self.state = "Navigate Intermediate Goal"
						
			elif self.state == "Navigate Intermediate Goal":
				(self.bearing, self.distance) = self.get_vector(self.x, self.y, self.intermediate_goal_x, self.intermediate_goal_y)
				#print("(", self.x, ",", self.y, ")")
				
				print("distance to goal:", self.distance)
				#print("bearing to goal:", self.bearing)
				
				error_distance = self.path_distance_pid.get_output(self.distance)
				error_theta = self.theta_pid.get_output(self.theta - self.bearing)
				
				#print("error distance & linear.x:", error_distance)
				#print("error theta & angular.z:", error_theta)
				
				twist.angular.z = error_theta
				twist.linear.x = error_distance
				
				if abs(error_distance) < 0.075:
					print("switch made")
					self.state = "Navigate Kick Position"

			elif self.state == "Navigate Kick Position":
				(self.bearing, self.distance) = self.get_vector(self.x, self.y, self.goal_x, self.goal_y)
				#print("distance to goal:", self.distance)
				#print("bearing to goal:", self.bearing)
				
				error_distance = self.path_distance_pid.get_output(self.distance)
				error_theta = self.theta_pid.get_output(self.theta - self.bearing)
			
				#print("error distance & linear.x:", error_distance)
				#print("error theta & angular.z:", error_theta)
				
				twist.angular.z = error_theta
				twist.linear.x = error_distance

				if abs(error_distance) < 0.075:
					print("switching to lineup")
					self.state = "Line Up"

			elif self.state == "Line Up":
				if self.bearing is None or self.distance is None:
					self.state = "Search"
				error_bearing = self.bearing_pid.get_output(self.bearing)
				twist.angular.z = error_bearing
				twist.linear.x = 0
				if abs(error_bearing) < 0.5:
					self.t = time.time();
					self.state = "Kick"

			elif self.state == "Kick":
				twist.angular.z = 0
				twist.linear.x = 1
				if time.time() - self.t > 1.5:
					self.state = "Search"
			self.x = 0
			self.y = 0
			self.theta = 0
			self.pub.publish(twist)
			rate.sleep()

rospy.init_node('robot_puppy')
robot = Puppy()
robot.run()
	
