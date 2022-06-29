#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from lab4_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""

# Calculate components needed for Forward Kinemtic
def skew(w):
	return np.array([[0, -w[2,0], w[1,0]],[w[2,0], 0, -w[0,0]],[-w[1,0], w[0,0], 0]])

def braket(T):
	w = T[:3].reshape(3,1)
	v = T[3:].reshape(3,1)
	W = skew(w)
	O = np.zeros((1,4))
	return np.concatenate((np.concatenate((W,v),axis = 1),O), axis = 0)

def rev(a,q):
	return np.concatenate((a,-skew(a).dot(q)), axis = 0)

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for a1~6 and q1~6, as well as the M matrix
	M = np.eye(4)
	S = np.zeros((6,6))

	a1 = np.array([[0],[0],[1]])
	q1 = np.array([[0],[0],[0]])

	a2 = np.array([[0],[1],[0]])
	q2 = np.array([[0],[0.12],[0.152]])

	a3 = np.array([[0],[1],[0]])
	q3 = np.array([[0.244],[0.12],[0.152]])

	a4 = np.array([[0],[1],[0]])
	q4 = np.array([[0.457],[0.027],[0.152]])

	a5 = np.array([[1],[0],[0]])
	q5 = np.array([[0.457],[0.11],[0.152]])

	a6 = np.array([[0],[1],[0]])
	q6 = np.array([[0.54],[0.11],[0.152]])

	M = np.array([[0, -1, 0, 0.54],[0, 0, -1, 0.257],[1, 0, 0, 0.207],[0, 0, 0, 1]])    

	S1 = rev(a1,q1)
	S2 = rev(a2,q2)
	S3 = rev(a3,q3)
	S4 = rev(a4,q4)
	S5 = rev(a5,q5)
	S6 = rev(a6,q6)
	S = np.concatenate((S1,S2,S3,S4,S5,S6), axis = 1)
	
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
	n = len(theta)
	for i in range(n):
		T = T.dot(expm(braket(S[:,i]*theta[i])))
	T = T.dot(M)
	# ==============================================================#
	
	print(str(T) + "\n")

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value



