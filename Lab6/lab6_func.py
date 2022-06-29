#!/usr/bin/env python
import numpy as np
import math
from scipy.linalg import expm, logm
from numpy import linalg
from lab6_header import *
"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for a1~6 and q1~6, as well as the M matrix
	M = np.eye(4)
	S = np.zeros((6,6))

	w1 = np.array([[0],[0],[1]])
	p1 = np.array([[0],[0],[0]])
	v1 = np.array([[0],[0],[0]])

	w2 = np.array([[0],[1],[0]])
	p2 = np.array([[0],[0],[0.152]])
	v2 = np.array([[-0.152],[0],[0]])

	w3 = np.array([[0],[1],[0]])
	p3 = np.array([[0.244],[0],[0.152]])
	v3 = np.array([[-0.152],[0],[0.244]])

	w4 = np.array([[0],[1],[0]])
	p4 = np.array([[0.457],[0],[0.152]])
	v4 = np.array([[-0.152],[0],[0.457]])

	w5 = np.array([[1],[0],[0]])
	p5 = np.array([[0],[0.110],[0.152]])
	v5 = np.array([[0],[0.152],[-.0120]])

	w6 = np.array([[0],[1],[0]])
	p6 = np.array([[0.540],[0],[0.152]])
	v6 = np.array([[-0.152],[0],[0.540]])


	M = np.array([[0, -1, 0, 0.540],[0, 0, -1, 0.250],[1, 0, 0, 0.2086],[0, 0, 0, 1]])    

	S1=np.array([[0],[0],[1],[0],[0],[0]])
	S2=np.array([[0],[1],[0],[-0.152],[0],[0]])
	S3=np.array([[0],[1],[0],[-0.152],[0],[0.244]])
	S4=np.array([[0],[1],[0],[-0.152],[0],[0.457]])
	S5=np.array([[1],[0],[0],[0],[0.152],[-0.120]])
	S6=np.array([[0],[1],[0],[-0.152],[0],[0.540]])



	S=[[0, 0, 0, 0, 1, 0], [0, 1, 1, 1, 0, 1], [1, 0, 0, 0, 0, 0], [0, -0.152, -0.152, -0.152, 0, 0], [0, 0, 0, 0, 0.152, 0], [0, 0, 0.244, 0.457, -0.120, 0.540]]
	
	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value 
	return_value = [None, None, None, None, None, None]

	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
	T = np.eye(4)

	M, S = Get_MS()

	S_ss1 = np.array([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 0],[0, 0, 0, 0]])
	S_ss2 = np.array([[0, 0, 1, -0.152], [0, 0, 0, 0], [-1, 0, 0, 0],[0, 0, 0, 0]])
	S_ss3 = np.array([[0, 0, 1, -0.152], [0, 0, 0, 0], [-1, 0, 0, 0.244],[0, 0, 0, 0]])
	S_ss4 = np.array([[0, 0, 1, -0.152], [0, 0, 0, 0], [-1, 0, 0, 0.457],[0, 0, 0, 0]])
	S_ss5 = np.array([[0, 0, 0, 0], [0, 0, -1, 0.152], [0, 1, 0, -0.110],[0, 0, 0, 0]])
	S_ss6 = np.array([[0, 0, 1, -0.152], [0, 0, 0, 0], [-1, 0, 0, 0.540],[0, 0, 0, 0]])

	T = linalg.multi_dot([expm(S_ss1*theta1),expm(S_ss2*theta2),expm(S_ss3*theta3),expm(S_ss4*theta4),expm(S_ss5*theta5),expm(S_ss6*theta6),M])






	# ==============================================================#
	
	print(str(T) + "\n")

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):

    # theta1 to theta6
	thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

	l01 = 0.152
	l02 = 0.120
	l03 = 0.244
	l04 = 0.093
	l05 = 0.213
	l06 = 0.083
	l07 = 0.083
	l08 = 0.082    
	l09 = 0.0535
	l10 = 0.059   # thickness of aluminum plate is around 0.006

	xgrip = xWgrip + 0.149
	ygrip = yWgrip - 0.149
	zgrip = zWgrip -0.016

	yaw = PI*yaw_WgripDegree/180

	xcen = xgrip - 0.0535*math.cos(yaw)
	ycen = ygrip - 0.0535*math.sin(yaw)
	zcen = zgrip
	

	# theta1
	thetas[0] = math.atan2(ycen, xcen) - math.asin(0.11/math.sqrt(xcen**2+ycen**2))        # Default value Need to Change

	# theta6
	thetas[5] = PI/2 + thetas[0] - yaw    # Default value Need to Change
 
	x3end = xcen - 0.083*math.cos(thetas[0]) + 0.110*math.sin(thetas[0])
	y3end = ycen - 0.083*math.sin(thetas[0]) - 0.110*math.cos(thetas[0])
	z3end = zcen + l08 + l10
	l3end = (x3end**2+y3end**2+(z3end-l01)**2)**0.5
	print((z3end-l01)/l3end)
	print((l03**2+l3end**2-l05**2)/(2*l03*l3end))
	thetas[1]= -math.asin((z3end-l01)/l3end) - math.acos((l03**2+l3end**2-l05**2)/(2*l03*l3end))
	thetas[2]= PI-math.acos((l03**2+l05**2-l3end**2)/(2*l03*l05))      # Default value Need to Change
	thetas[3]= - thetas[1]-thetas[2]  # Default value Need to Change, need + (0.5*PI) for compensation
	thetas[4]= -PI/2

	print("theta1 to theta6: " + str(thetas) + "\n")

	return lab_fk(float(thetas[0]), float(thetas[1]), float(thetas[2]), \
		          float(thetas[3]), float(thetas[4]), float(thetas[5]) )
