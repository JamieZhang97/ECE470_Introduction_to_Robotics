#!/usr/bin/env python

import cv2
import numpy as np 

"""
To init blob search params, will be init (called) in the ImageConverter class
"""
def blob_search_init():

	# Setup SimpleBlobDetector parameters.
	params = cv2.SimpleBlobDetector_Params()

	################# Your Code Start Here #################

	# Filter by Color 
	params.filterByColor = False


	# Filter by Area.
	params.filterByArea = True    
	params.minArea = 15
	params.maxArea = 1000
	# Filter by Circularity
	params.filterByCircularity = True
	params.minCircularity = 0.3
	params.maxCircularity = 1

	# Filter by Inerita
	params.filterByInertia = False


	#Filter by Convexity
	params.filterByConvexity = False
	# params.minConvexity = 0.87

	# params.minThreshold = 100
	# params.maxThreshold = 5000
	# params.blobColor = 255


	################## Your Code End Here ##################

	# Create a detector with the parameters
	blob_detector = cv2.SimpleBlobDetector_create(params)

	return blob_detector


"""
To find blobs in an image, will be called in the callback function of image_sub subscriber
"""
def blob_search(image, detector):

	# Convert the color image into the HSV color space
	hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


	############################ Your Code Start Here ############################

	# Find lower & upper for orange
	
	lower =(1,100,100)      # blue lower
	upper = (15,255,255)   # blue upper
 
	############################# Your Code End Here #############################


	# Define a mask using the lower and upper bounds of the orange color 
	mask_image = cv2.inRange(hsv_image, lower, upper)

	crop_top_row = 100
	crop_bottom_row = 350
	crop_top_col = 150
	crop_bottom_col = 500

	crop_image = mask_image[crop_top_row:crop_bottom_row, crop_top_col:crop_bottom_col]

	blob_image_center = []

	############################ Your Code Start Here ############################

	# Call opencv simpleBlobDetector functions here to find centroid of all large enough blobs in 
	# crop_image. Make sure to add crop_top_row and crop_top_col to the centroid row and column found

	# Make sure this blob center is in the full image pixels not the cropped image pixels
	keypoints = detector.detect(crop_image)
	#print keypoints
	
	centroid = []
	for i in range(len(keypoints)):
		centroid.append(list(keypoints[i].pt))

	for k in range(len(centroid)):
		centroid[k][0]+=crop_top_col
		centroid[k][1]+=crop_top_row

	for i in range(len(keypoints)):
		keypoints[i].pt=tuple(centroid[i])


	
			
	
		#return[blob_image_center]

	im_with_keypoints = cv2.drawKeypoints(image, keypoints, np.array([]), 
			 (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

	for i in range(len(keypoints)):
		blob_image_center.append(str(centroid[i][0])+" "+str(centroid[i][1])) #str(x)+' '+str(y)
		cv2.circle(im_with_keypoints,(int(centroid[i][0]),int(centroid[i][1])),2, (0, 0, 255), -1)
			
	# for k in keypoints:
		
	#cv2.imshow("keypoints", im_with_keypoints)

	
	


		




		#obj.Proxy.image = im_with_keypoints
		#for k in keypoints:
		#x = int(round(x))
		#cv2.circle(image, (x,y),4,0,5)
		#	image[y,x]=255
		#	im[y,x]=0
		#obj.Proxy.img = cv2.cvtColor(im, cv2.COLOR_GRAY2BGR)
	# Draw centers on each blob, append all the centers to blob_image_center as string in format "x y"
		#else:
		#for k in keypoints:
	##		cv2.circle(image2, (x,y),4,(255,0,0),5)
	#		cv2.circle(image2, (x,y),4,(0,0,0),5)
	#		image2[y,x]=(255,0,0)
	#	obj.Proxy.img = image2


		

	############################# Your Code End Here #############################

	# Draw small circle at pixel coordinate crop_top_col, crop_top_row so you can move a color
	# under that pixel location and see what the HSV values are for that color. 
	image = cv2.circle(image, (int(crop_top_col), int(crop_top_row)), 3, (0, 0, 255), -1)
	print('H,S,V at pixel ' + str(crop_top_row) + ' ' + str(crop_top_col) + ' ' + str(hsv_image[crop_top_row,crop_top_col]))	

	cv2.namedWindow("Maze Window")
	cv2.imshow("Maze Window", im_with_keypoints)

	cv2.namedWindow("MaskImage Window")
	cv2.imshow("MaskImage Window", mask_image)

	cv2.namedWindow("Crop Window")
	cv2.imshow("Crop Window", crop_image)

	cv2.waitKey(2)

	
	
	# blob = []
 #    	for i in keypoints:
 #    		x = i.pt[0]; y = i.pt[1]
 #    		blob.append('x y')
 #    	blob_image_center = blob
	print(blob_image_center)
	return blob_image_center
	#return image

	#return(blob_image_center.format(x, y))