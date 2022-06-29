#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from lab4_header import *
from numpy import linalg 

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

	S_ss1 = np.array([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 0],[0, 0, 0, 0]])
	S_ss2 = np.array([[0, 0, 1, -0.152], [0, 0, 0, 0], [-1, 0, 0, 0],[0, 0, 0, 0]])
	S_ss3 = np.array([[0, 0, 1, -0.152], [0, 0, 0, 0], [-1, 0, 0, 0.244],[0, 0, 0, 0]])
	S_ss4 = np.array([[0, 0, 1, -0.152], [0, 0, 0, 0], [-1, 0, 0, 0.457],[0, 0, 0, 0]])
	S_ss5 = np.array([[0, 0, 0, 0], [0, 0, -1, 0.152], [0, 1, 0, -0.110],[0, 0, 0, 0]])
	S_ss6 = np.array([[0, 0, 1, -0.152], [0, 0, 0, 0], [-1, 0, 0, 0.540],[0, 0, 0, 0]])

	#S_ss = np.array([[S_ss1][S_ss2][S_ss3][S_ss4][S_ss5][S_ss6]])

	global S_ss1
	global S_ss2
	global S_ss3
	global S_ss4
	global S_ss5
	global S_ss6

	
	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value 
	return_value = [None, None, None, None, None, None]

	# =========== Implement joint angle to encoder expressions here ===========
	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
	T = np.eye(4)
	
	
	
	M, S = Get_MS()

	# n = len(theta)
	# for i in range(n):
	# 	T = T.dot(expm(S_ss(i,1)*theta[i]))
	# T = T.dot(M)
 # 	A = np.dot(expm(S_ss1*theta[0]+S_ss2*theta[1]+S_ss3*theta[2]+S_ss4*theta[3]+S_ss5*theta[4]+S_ss6*theta[5]))
	# T = np.dot(A,M)
	
	# 
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



