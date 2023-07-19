#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, TransformStamped
from beginner_tutorials.msg import BallLocation
from kobuki_msgs.msg import BumperEvent
import time
import numpy as np
from math import atan2, sin, cos, pi, sqrt
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import tf2_ros

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
		self.x = self.y = self.goal_x = self.goal_y = self.ball_x = self.ball_y = self.side_x = self.side_y = self.theta = self.t = self.goal_magnitude = self.prev_bearing = 0.0
		
		self.bearing = self.distance = None
		self.straight_to_kick = self.just_kicked = False
		self.approaches = ""
		self.state = "Locate Goal"
		
		self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)					
		rospy.Subscriber('/ball_detector/ball_location', BallLocation, self.handle_ball_location)
		rospy.Subscriber('/odom', Odometry, self.handle_pose)
		rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumped)
		
		self.bearing_pid = PID(320,2/320,0,0,0.667)
		self.distance_pid = PID(1.5,-0.5,0,0,0.5)
		self.theta_pid = PID(0, 1, 0, 0, 0.75)
		self.path_distance_pid = PID(0, -0.25, 0, 0, 0.75)
		
		self.listener = tf2_ros.Buffer()
		tf2_ros.TransformListener(self.listener)
	
	def bumped(self, msg):        
		self.state = "Back up"
		self.t = time.time()
		
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

	def get_vector(self, from_x, from_y, to_x, to_y):
		bearing = atan2(to_y - from_y, to_x - from_x)
		distance = sqrt((from_x - to_x)**2 + (from_y - to_y)**2)
		return (bearing, distance)
	
	def get_error_bearing(self, theta, bearing):
		bearing = theta - bearing
		bearing = ((bearing + pi) % 2*pi) - pi
		return bearing
	
	def get_pid_outputs(self, goal_distance, goal_bearing, distance_pid, bearing_pid, twist):
		linear = twist.linear.x = distance_pid.get_output(goal_distance)
		angular = twist.angular.z =  bearing_pid.get_output(goal_bearing)
		return (linear, angular)
	
	def run(self):
		rate = rospy.Rate(10)
		twist = Twist()
		while not rospy.is_shutdown():
			if self.state == "Locate Goal":
				if self.goal_x != 0 and self.goal_y != 0:
					self.state = "Search"
					
			elif self.state == "Search":
				twist.linear.x = 0
				twist.angular.z = 0.75
				if self.bearing is not None and self.distance is not None:
					self.state = "Approach"
					
			elif self.state == "Approach":
				if self.bearing is None or self.distance is None:
					self.state = "Search"
				else:
					(linear_error, _) = self.get_pid_outputs(self.distance, self.bearing, self.distance_pid, self.bearing_pid, twist)
					
					if abs(linear_error) < 0.1:
						self.ball_x = self.x + 1.5*cos(self.theta)
						self.ball_y = self.y + 1.5*sin(self.theta)
						
						(_, self.goal_magnitude) = self.get_vector(self.goal_x, self.goal_y, self.ball_x, self.ball_y)
						
						unit_vec_x = self.ball_x - self.goal_x
						unit_vec_x /= self.goal_magnitude
						unit_vec_y = self.ball_y - self.goal_y
						unit_vec_y /= self.goal_magnitude
						
						self.kick_x = self.ball_x + 1.5*unit_vec_x
						self.kick_y = self.ball_y + 1.5*unit_vec_y
					
						#print("Goal (x,y): (",self.goal_x,", ",self.goal_y,")")
						#print("Ball (x,y): (",self.ball_x,", ",self.ball_y,")")
						#print("Kick (x,y): (",self.kick_x,", ",self.kick_y,")")

						(_, distance_to_kick) = self.get_vector(self.x, self.y, self.kick_x, self.kick_y)
						(_, distance_to_ball) = self.get_vector(self.x, self.y, self.ball_x, self.ball_y)
						
						self.theta_pid = PID(0, 1.0, 0, 0, 0.75)
						
						if abs(distance_to_kick) < abs(distance_to_ball):
							self.straight_to_kick = True
							self.theta_pid = PID(0, 1.0, 0, 0, 1)
							self.approaches = ""
							self.state = "Navigate Kick Position"
							
						else:
							left_side_x = self.x + 1.5*sqrt(2)*cos(self.theta + pi/4)
							left_side_y = self.y + 1.5*sqrt(2)*sin(self.theta + pi/4)
							
							right_side_x = self.x + 1.5*sqrt(2)*cos(self.theta - pi/4)
							right_side_y = self.y + 1.5*sqrt(2)*sin(self.theta - pi/4)
							
							#print("Left side (x,y): (",left_side_x,", ",left_side_y,")")
							#print("Right side (x,y): (",right_side_x,", ",right_side_y,")")
							#print("Ball (x,y): (",self.ball_x,", ",self.ball_y,")")
							
							(_, left_to_kick) = self.get_vector(left_side_x, left_side_y, self.kick_x, self.kick_y)
							(_, right_to_kick) = self.get_vector(right_side_x, right_side_y, self.kick_x, self.kick_y)
							
							if abs(left_to_kick) < abs(right_to_kick):
								self.side_x = left_side_x
								self.side_y = left_side_y
								self.approaches = "left"
								self.state = "Navigate Intermediate Goal"
							else:
								self.side_x = right_side_x
								self.side_y = right_side_y
								self.approaches = "right"
								self.state = "Navigate Intermediate Goal"
						
			elif self.state == "Navigate Intermediate Goal":
			
				(goal_bearing, goal_distance) = self.get_vector(self.x, self.y, self.side_x, self.side_y)
				error_bearing = self.get_error_bearing(self.theta, goal_bearing)
				(linear_error, _) = self.get_pid_outputs(goal_distance, error_bearing, self.path_distance_pid, self.theta_pid, twist)
				
				if abs(linear_error) < 0.15:
					if self.approaches == "left":
						self.theta_pid = PID(0, 1.0, 0, 0, 0.75)
					self.t = time.time()
					self.state = "Navigate Kick Position"

			elif self.state == "Navigate Kick Position":
			
				twist.linear.x = 0
				if self.approaches == "left":
					twist.angular.z = -1
				elif self.approaches == "right":
					twist.angular.z = 1
					
				if time.time() - self.t > 1 or self.straight_to_kick == True:
				
					(goal_bearing, goal_distance) = self.get_vector(self.x, self.y, self.kick_x, self.kick_y)
					error_bearing = self.get_error_bearing(self.theta, goal_bearing)
					(linear_error, _) = self.get_pid_outputs(goal_distance, error_bearing, self.path_distance_pid, self.theta_pid, twist)
						
					if abs(linear_error) < 0.15:
						self.state = "Line Up"
						self.t = time.time()

			elif self.state == "Line Up":
				twist.linear.x = 0
				if self.approaches == "left":
					twist.angular.z = -1
					
					if time.time() - self.t > 1.4:
						if self.bearing is not None:
							self.prev_bearing = -self.bearing
							error_bearing = self.bearing_pid.get_output(self.bearing)
							twist.angular.z = error_bearing
							if abs(error_bearing) < 0.5:
								self.t = time.time()
								self.state = "Kick"
						elif self.bearing is None and self.prev_bearing is not None:
							error_bearing = self.bearing_pid.get_output(self.prev_bearing)
							twist.angular.x = error_bearing
							if abs(error_bearing) < 0.5:
								self.t = time.time()
								self.state = "Kick"
						elif time.time() - self.t > 5:
							self.state = "Search"
							self.just_kicked = False
							
				elif self.approaches == "right":
					twist.angular.z = 1
					if time.time() - self.t > 1.4:
						if self.bearing is not None:
							self.prev_bearing = self.bearing
							error_bearing = self.bearing_pid.get_output(self.bearing)
							twist.angular.z = error_bearing
							if abs(error_bearing) < 0.5:
								self.t = time.time()
								self.state = "Kick"
						elif self.bearing is None and self.prev_bearing is not None:
							error_bearing = self.bearing_pid.get_output(self.prev_bearing)
							twist.angular.x = error_bearing
							print(error_bearing, "prev_bearing used")
							if abs(error_bearing) < 0.5:
								self.t = time.time()
								self.state = "Kick"
						elif time.time() - self.t > 5:
							self.state = "Search"
							self.just_kicked = "False"
							
				elif self.straight_to_kick == True:
					if self.bearing is not None:
						self.prev_bearing = self.bearing
						error_bearing = self.bearing_pid.get_output(self.bearing)
						twist.angular.z = error_bearing
						if abs(error_bearing) < 0.5:
							self.t = time.time()
							self.state = "Kick"
					elif self.bearing is None and self.prev_bearing is not None:
						error_bearing = self.bearing_pid.get_output(self.prev_bearing)
						twist.angular.z = error_bearing
						if abs(error_bearing) < 0.5:
							self.t = time.time()
							self.state = "Kick"
					elif time.time() - self.t > 5:
						self.state = "Search"
						self.just_kicked = False

			elif self.state == "Kick":
				twist.angular.z = 0
				twist.linear.x = 1
				if time.time() - self.t > 2:
					self.just_kicked = True
					self.state = "Search"
					self.t = time.time()
					
			elif self.state == "Back up":
				twist.angular.z = 0
				twist.linear.x = -0.8
				if time.time() - self.t > 1.15:
					self.just_kicked = False
					self.state = "Search"
					
			#print(self.state)
			
			self.pub.publish(twist)
			try:
				trans = self.listener.lookup_transform('odom', 'ar_marker_0', rospy.Time())
				self.goal_x = trans.transform.translation.x
				self.goal_y = trans.transform.translation.y
			except tf2_ros.LookupException:
				pass
			except tf2_ros.ConnectivityException:
				pass
			except tf2_ros.ExtrapolationException:
				pass
			rate.sleep()

rospy.init_node('robot_puppy')
robot = Puppy()
robot.run()
