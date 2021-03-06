#!/usr/bin/env python

import sys
import cv2
import copy
import time
import numpy as np 
import math

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from lab3_func import blob_search_init, blob_search


# Params for camera calibration
theta = 0 
beta = 0
tx = 0
ty = 0



class ImageConverter:

	def __init__(self, SPIN_RATE):

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/cv_camera_node/image_raw", Image, self.image_callback)
		self.loop_rate = rospy.Rate(SPIN_RATE)
		self.detector = blob_search_init()

		# Check if ROS is ready for operation
		while(rospy.is_shutdown()):
			print("ROS is shutdown!")

# Params for camera calibration
# theta = 0 
# beta = 0
# tx = 0
# ty = 0

# #######################################################################################


# class ImageConverter:

# 	def __init__(self, SPIN_RATE):

# 		self.bridge = CvBridge()
# 		self.image_sub = rospy.Subscriber("/cv_camera_node/image_raw", Image, self.image_callback)
# 		self.coord_pub = rospy.Publisher("/coord_center", String, queue_size=10)
# 		self.loop_rate = rospy.Rate(SPIN_RATE)
# 		self.detector = blob_search_init()

# 		# Check if ROS is ready for operation
# 		while(rospy.is_shutdown()):
# 			print("ROS is shutdown!")


	# def image_callback(self, data):

	# 	global theta
	# 	global beta
	# 	global tx
	# 	global ty

	# 	try:
	# 		# Convert ROS image to OpenCV image
	# 		raw_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
	# 	except CvBridgeError as e:
	# 		print(e)

	# 	# Flip the image 180 degrees
	# 	cv_image = cv2.flip(raw_image, -1)
	def image_callback(self, data):

		global theta
		global beta
		global tx
		global ty

		try:
			# Convert ROS image to OpenCV image
			raw_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		# Flip the image 180 degrees
		cv_image = cv2.flip(raw_image, -1)

		# Draw a black line on the image
		cv2.line(cv_image, (0,50), (640,50), (0,0,0), 5)

		# cv_image is normal color image
		blob_image_center = blob_search(cv_image, self.detector)

		# Given world coordinate (xw, yw)
		xw = 0.2875
		yw = 0.1125	

		# Only two blob center are found on the image
		if(len(blob_image_center) == 2):

			x1 = int(float(blob_image_center[0].split()[0]))
			y1 = int(float(blob_image_center[0].split()[1]))
			x2 = int(float(blob_image_center[1].split()[0]))
			y2 = int(float(blob_image_center[1].split()[1]))

			print("Blob Center 1: ({0}, {1}) and Blob Center 2: ({2}, {3})".format(x1, y1, x2, y2))

			################################# Your Code Start Here ################################# 

			# Calculate beta, tx and ty, given x1, y1, x2, y2

			
			Or = 240
			Oc = 320			
			beta = (10.*abs(x1-x2))

			if(x1>x2):
				tx=(y2-Or)/beta-xw
				ty=(x2-Oc)/beta-yw
				
				xw = ((y2-Or)/beta-tx)*np.cos(theta)+((x2-Oc)/beta-ty)*np.sin(theta)
				yw = ((y2-Or)/beta-tx)*(-np.sin(theta))+((x2-Oc)/beta-ty)*np.cos(theta)

			else:
				tx=(y1-Or)/beta-xw
				ty=(x1-Oc)/beta-yw		
				
				xw = ((y1-Or)/beta-tx)*np.cos(theta)+((x1-Oc)/beta-ty)*np.sin(theta)
				yw = ((y1-Or)/beta-tx)*(-np.sin(theta))+((x1-Oc)/beta-ty)*np.cos(theta)

			# r = x2-x1
			# c = y2-y1
			
			# xw = ((x-240-beta*tx)*math.cos(theta)+(y-320-beta*ty)*math.sin(theta))/(beta*1.0)
			# yw = ((y-320-beta*ty)*math.cos(theta)+(x-240-beta*tx)*math.sin(theta))/(beta*1.0)


			#beta = (10*abs(r))
			# xc = float((r-Or)/beta)
			# yc = float((c-Oc)/beta)
			# tx = xw-xc
			# ty = yw-yc
	





			################################## Your Code End Here ################################## 

			print("theta = {0}\nbeta = {1}\ntx = {2}\nty = {3}\n".format(theta, beta, tx, ty))
			#print(xw, yw)

		else:
			print("No Blob found! ")


def main():

	SPIN_RATE = 20 # 20Hz

	rospy.init_node('lab3ImageCalibrationNode', anonymous=True)

	ic = ImageConverter(SPIN_RATE)

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down!")

	cv2.destroyAllWindows()


if __name__ == '__main__':
	main()