#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
		q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') #theta_i, angle rotation

		d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') #link off-sets

		a0, a1, a2, a3, a4, a5, a6= symbols('a0:7') #link lenghts

		alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') #Twist angles
	#
	#
	# Create Modified DH parameters
		d02 = 0.75
		d35 = 1.5
		dg = 0.303 #gripper link off-set
		a02 = 0.35
		a23 = 1.25
		a34 = -0.054
		anp = pi/2
		ann = - pi/2
	#
	# Define Modified DH Transformation matrix
		dht = {alpha0: 	 0,   a0:  	 0,    d1:  d02,	q1: 	  q1,
		 	   alpha1: ann,   a1:  a02,    d2:    0,    q2: ann + q2,
		 	   alpha2: 	 0,   a2:  a23,    d3:    0,	q3:		  q3,
		 	   alpha3: ann,   a3:  a34,    d4:  d35,	q4:		  q4,
		 	   alpha4: anp,   a4:  	 0,    d5:    0,	q5: 	  q5,
		 	   alpha5: ann,   a5:  	 0,    d6:    0,	q6: 	  q6,
		 	   alpha6: 	 0,   a6:  	 0,    d7:   dg,    q7:  	   0}
	#
	#
	# Create individual transformation matrices
		def Ind_transform(q, alpha, d, a): #individual transform matrix

			T0_N = Matrix([[        cos(q),           -sin(q),           0,             a],
            	   	   [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
               		   [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
               		   [                 0,                 0,           0,            1]])

			return T0_N

		# Individual transform for each link
		T0_1 = Ind_transform(q1, alpha0, d1, a0).subs(dht)
		T1_2 = Ind_transform(q2, alpha1, d2, a1).subs(dht)
		T2_3 = Ind_transform(q3, alpha2, d3, a2).subs(dht)
		T3_4 = Ind_transform(q4, alpha3, d4, a3).subs(dht)
		T4_5 = Ind_transform(q5, alpha4, d5, a4).subs(dht)
		T5_6 = Ind_transform(q6, alpha5, d6, a5).subs(dht)
		T6_G = Ind_transform(q7, alpha6, d7, a6).subs(dht)


		# Extract rotation matrices from the transformation matrices
		T0_2 = simplify(T0_1 * T1_2) #base link to link 2
		T0_3 = simplify(T0_2 * T2_3) #base link to link 3
		T0_4 = simplify(T0_3 * T3_4) #base link to link 4
		T0_5 = simplify(T0_4 * T4_5) #base link to link 5
		T0_6 = simplify(T0_5 * T5_6) #base link to link 6
		T0_G = simplify(T0_6 * T6_G) ##base link to link G or end effector
	#
	#
		# Extract rotation matrices from the transformation matrices

		T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G ##base link to link G or end effector

		# Correction required on the end effector to get the real position from kr210.urdf.xacro
		# Rotation on the z-axis by pi radiants or 180 grees
		R_z = Matrix([[             cos(pi),            -sin(pi),            0,              0],
           [                        sin(pi),            cos(pi),             0,              0],
           [                        0,                  0,                   1,              0],
           [                        0,                  0,                   0,              1]])

		# then rotate around y-axis by -pi/2 radiants or -90 degrees
		R_y = Matrix([[             cos(-pi/2),         0,                   sin(-pi/2),     0],
           [                        0,                  1,                   0,              0],
           [                        -sin(-pi/2),        0,                   cos(-pi/2),     0],
           [                        0,                  0,                   0,              1]])

		#Calculate total correction factor of the urdf file on the end effector
		R_corr = simplify(R_z * R_y)

		# Initialize service response
		joint_trajectory_list = []
		for x in xrange(0, len(req.poses)):
		### IK code starts here
			joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation

			px = req.poses[x].position.x
			py = req.poses[x].position.y
			pz = req.poses[x].position.z

			(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
				[req.poses[x].orientation.x, req.poses[x].orientation.y,
					req.poses[x].orientation.z, req.poses[x].orientation.w])

			# Construct end efector base on yaw, pich and roll
			Rw, Pt, Yw = symbols('Rw Pt Yw')

			R_roll =  Matrix([[ 	   1,        0,     	  0],
                      	  	  [        0,  cos(Rw),    -sin(Rw)],
                      	  	  [        0,  sin(Rw),  	cos(Rw)]]) #Roll

			R_pitch = Matrix([[  cos(Pt),        0,  	sin(Pt)],
                      	  	  [        0,        1,           0],
                      	  	  [ -sin(Pt),        0,  	cos(Pt)]]) #Pitch

			R_yaw =   Matrix([[ cos(Yw),  -sin(Yw),           0],
                      	  	  [ sin(Yw),   cos(Yw),           0],
                      	  	  [   	  0,         0,           1]])  #Yaw

        	## Extract end effector rotation matrices
        	R0_G = simplify(R_yaw * R_pitch * R_roll)
        	R0_G = simplify(R0_G * R_corr[0:3, 0:3])
        	R0_G = R0_G.subs({Rw: roll ,Pt: pitch, Yw: yaw})

        	# End efector current position
        	EE = Matrix([[px], [py], [pz]])

		# From the form to calculate the wrist center: W = p - (d6 + l) * n
        	WC = EE - dg * R0_G[:,2]

        	############# Calculate Theta 1, 2 and 3 #############
        	#
        	theta1 = atan2(WC[1],WC[0])

        	#calculate side b with cosin law
        	#														O3_O4			     O4_WC		
    		side_a = 1.501 # space between O2 and WC = (sqrt(pow(0.96) + pow(0.054))) +   0.54 
    		side_b = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35),2) + pow((WC[2] - 0.75),2))
    		side_c = 1.25 # O2_O3

    		#calculate each angle
    		angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c))
    		angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c))
    		angle_c = acos((side_a * side_a + side_b * side_b - side_c * side_c) / (2 * side_a * side_b))

    		#caclulate theta 2
    		theta2 = pi / 2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
    		#caclulate theta 3
    		theta3 = pi / 2 - (angle_b + 0.036)

    		# calculate rotation matrix from link o to 3 using theta 1, 2 and 3
    		R0_3 = T0_3[0:3, 0:3] #extracting rotation propertis
    		R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

    		R3_6 = R0_3.inv("LU") * R0_G # Rotation matrix from link 3 to end efecotr

    		########### Calculate Theta 4, 5, and 6 ##############
    		theta4 = atan2(R3_6[2,2], -R3_6[0,2])
    		theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
    		theta6 = atan2(-R3_6[1,1], R3_6[1,0])



		# Populate response for the IK request
		# In the next line replace theta1,theta2...,theta6 by your joint angle variables
		joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
		joint_trajectory_list.append(joint_trajectory_point)



		rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
		return CalculateIKResponse(joint_trajectory_list)

	    #
	    #
	    # Calculate joint angles using Geometric IK method
	    #
	    #
            ###

		# Populate response for the IK request
		# In the next line replace theta1,theta2...,theta6 by your joint angle variables
		joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
		joint_trajectory_list.append(joint_trajectory_point)

		rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
		return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
