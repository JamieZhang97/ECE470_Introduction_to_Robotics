#!/usr/bin/env python

import cv2
import numpy as np

# ============================= Student's code starts here ===================================

# Params for camera calibration
theta = 0.0 
beta = 750.0
tx = 0.260666666667 
ty = 0.119166666667


# Function that converts image coord to world coord
def IMG2W(x,y):

	Or = 240
	Oc = 320

			
	xw_cal = ((y-Or)/beta-tx)*np.cos(theta)+((x-Oc)/beta-ty)*np.sin(theta)+0.546
	yw_cal = ((y-Or)/beta-tx)*(-np.sin(theta))+((x-Oc)/beta-ty)*np.cos(theta)+0.232



	# xw_cal = (((y-240)/beta-tx)*np.cos(theta)+((x-320)/beta-ty)*np.sin(theta))+0.15
	# yw_cal = (((y-240)/beta-tx)*-np.sin(theta)+((x-320)/beta-ty)*np.cos(theta))-0.15
	return xw_cal, yw_cal

def blob_search(image_raw, color):

	# Setup SimpleBlobDetector parameters.
	params = cv2.SimpleBlobDetector_Params()

	params.minDistBetweenBlobs = 20.0
	# Filter by Color 
	params.filterByColor = False

	# Filter by Area.
	params.filterByArea = True   
	params.minArea = 300
	params.maxArea = 1400
	# Filter by Circularity
	params.filterByCircularity = False
	#params.minCircularity = 0.785

	# Filter by Inerita
	params.filterByInertia = False


	# Filter by Convexity
	params.filterByConvexity = False
	#params.minConvexity = 0.87


	# Create a detector with the parameters
	detector = cv2.SimpleBlobDetector_create(params)

	# Crop the image
	image = image_raw[100:330, 100:540].copy()

	# Convert the image into the HSV color space
	hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)



	lower_red = (1, 120, 100) #red lower
	upper_red = (15, 255, 255) #red upper

	lower_green = (50, 43, 46) #green lower
	upper_green = (70, 255, 255) #green upper 

	# Define a mask using the lower and upper bounds of the target color 
	if (color == 'red'):
		mask_image = cv2.inRange(hsv_image, lower_red, upper_red)

	if (color == 'green'):
		mask_image = cv2.inRange(hsv_image, lower_green, upper_green)

	keypoints = detector.detect(mask_image)

	#print keypoints
	
	
	im_with_keypoints = cv2.drawKeypoints(image, keypoints, np.array([]), 
			 (0, 255, 0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

	
	#im_with_keypoints = cv2.drawKeypoints(image, keypoints, ???)

	# Find blob centers in the image coordinates
	blob_image_center = []

	for i in keypoints:
		im_with_keypoints = cv2.circle(im_with_keypoints,(int(i.pt[0]), int(i.pt[1])), 2, (0, 255, 0), -1)
		xy_center = [int(i.pt[0]+100), int(i.pt[1]+100)]
		blob_image_center.append(xy_center)

	xw_yw = []

	if(len(blob_image_center) == 0):
		print("No block found!")
	else:
		for i in range(len(blob_image_center)):
			x = blob_image_center[i][0]
			y = blob_image_center[i][1]
			(xw_block, yw_block) = IMG2W(x,y)
			xw_yw.append([xw_block,yw_block])


	cv2.namedWindow("Camera View")
	cv2.imshow("Camera View", im_with_keypoints)

	cv2.namedWindow("MaskImage Window")
	cv2.imshow("MaskImage Window", mask_image)

	cv2.waitKey(2)

	return xw_yw