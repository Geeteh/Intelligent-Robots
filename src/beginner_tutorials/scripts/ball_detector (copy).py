#!/usr/bin/env python3
import roslib
roslib.load_manifest('beginner_tutorials')
import rospy
import cv2
from sensor_msgs.msg import Image, LaserScan
from beginner_tutorials.msg import BallLocation
from cv_bridge import CvBridge, CvBridgeError
import numpy
import math
class Detector:
	def __init__(self):
		#self.impub = rospy.Publisher('/ball_detector/image', Image, queue_size=1)
		self.locpub = rospy.Publisher('/ball_detector/ball_location', BallLocation, queue_size=1)
		self.bridge = CvBridge()
		self.bearing = -1
		self.distance = -1
		rospy.Subscriber('/camera/rgb/image_raw', Image, self.handle_image)
		rospy.Subscriber('/scan', LaserScan, self.handle_scan)

	def handle_image(self, msg):
		try:
			image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		except CvBridgeError:
			print("Cannot convert ROS message to OpenCV image.")
		
		(rows, columns, channels) = image.shape
		
		xcoord_sum = 0
		yellow_pixels = 0
		for i in range(0, columns, 10):
			for j in range(260, 450):
				if image[j,i,0] > 50 and image[j,i,0] < 70 and image[j,i,1] > 150 and image[j,i,1] < 170 and image[j,i,2] > 180 and image[j,i,2] < 200:
					#image[j,i,0] = 0
					#image[j,i,1] = 0
					#image[j,i,2] = 0
					xcoord_sum += i
					yellow_pixels = yellow_pixels + 1
			
			
		if yellow_pixels != 0:
			self.bearing = int(xcoord_sum/yellow_pixels)
		
		else:
			self.bearing = -1
		
		#self.impub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))

	def handle_scan(self, msg):
		if self.bearing != -1:
			if len(msg.ranges) > self.bearing and not numpy.isnan(msg.ranges[-self.bearing]):
				self.distance = msg.ranges[-self.bearing]
				
			else:
				self.distance = -1
				self.bearing = -1
		else:
			self.distance = -1
			self.bearing = -1
			
		#if (numpy.isnan(self.distance)):
		#	self.distance = -1
		#	self.bearing = -1

	def start(self):
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			location = BallLocation()
			location.bearing = self.bearing
			location.distance = self.distance
			self.locpub.publish(location)
			rate.sleep()

rospy.init_node('ball_detector')
detector = Detector()
detector.start()
