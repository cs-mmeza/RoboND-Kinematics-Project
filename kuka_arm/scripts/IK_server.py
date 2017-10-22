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
		q1, q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') #theta_i, angle rotation

		d1, d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') #link off-sets

		a0, a1, a2, a3, a4, a5, a6, a6 = symbols('a0:7') #link lenghts

		alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') #Twist angles
	#
	#   
	# Create Modified DH parameters
		d02 = 0.75
		d35 = 1.5
		dg = 0.2305
		a02 = 0.35
		a23 = 1.25
		a34 = -0.054
    	anp = pi/2
    	ann = - pi/2
	#            
	# Define Modified DH Transformation matrix
		s = {alpha0: 0,   a0:  0,    d1:  d02, 
		 	alpha1: ann, a1:  a02,  d2:  0,   q2: ann,
		 	alpha2: 0,   a2:  a23,  d3:  0, 	 
		 	alpha3: ann, a3:  a34,  d4:  d35, 
		 	alpha4: anp, a4:  0,    d5:  0, 	 
		 	alpha5: ann, a5:  0,    d6:  0, 	 
		 	alpha6: 0,   a6:  0,    d7:  0,   q7: 0}
	#
	#
	# Create individual transformation matrices
		T0_1 = Matrix([[         cos(q1),            -sin(q1),            0,              a0],
               	   [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
               	   [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
               	   [                   0,                   0,            0,               1]])
		T0_1 = T0_1.subs(s)

		T1_2 = Matrix([[         cos(q2),            -sin(q2),            0,              a1],
               	   [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
                   [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
                   [                   0,                   0,            0,               1]])
		T1_2 = T1_2.subs(s)

		T2_3 = Matrix([[         cos(q3),            -sin(q3),            0,              a2],
                   [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
                   [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
                   [                   0,                   0,            0,               1]])
		T2_3 = T2_3.subs(s)

		T3_4 = Matrix([[         cos(q4),            -sin(q4),            0,              a3],
                   [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
                   [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
                   [                   0,                   0,            0,               1]])
		T3_4 = T3_4.subs(s)

		T4_5 = Matrix([[         cos(q5),            -sin(q5),            0,              a4],
                   [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
                   [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
                   [                   0,                   0,            0,               1]])
		T4_5 = T3_4.subs(s)
	
		T5_6 = Matrix([[         cos(q6),            -sin(q6),            0,              a5],
                   [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
                   [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
                   [                   0,                   0,            0,               1]])
		T5_6 = T3_4.subs(s)

		T6_G = Matrix([[         cos(q7),            -sin(q7),            0,              a6],
                   [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
                   [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
                   [                   0,                   0,            0,               1]])
		T6_G = T3_4.subs(s)
	#
	#
	# Extract rotation matrices from the transformation matrices
		T0_2 = simplify(T0_1 * T1_2) #base link to link 2
		T0_3 = simplify(T0_2 * T2_3) #base link to link 3	
		T0_4 = simplify(T0_3 * T3_4) #base link to link 4
		T0_5 = simplify(T0_4 * T4_5) #base link to link 5
		T0_6 = simplify(T0_5 * T5_6) #base link to link 6
		T0_G = simplify(T0_6 * T6_G) ##base link to link G

	#
	#
    #   Correction needed to account of orientation difference between definition of gripper link
    #   in URDF vs DH convertion
        R_z = Matrix([[     cos(np.pi),     -sin(np.pi),    		  0,   0],
        			  [     sin(np.pi),      cos(np.pi),    		  0,   0],
        			  [              0, 	   	      0,    		  1,   0],
        			  [              0, 		      0,    		  0,   1]])

        R_y = Matrix([[  cos(-np.pi/2),               0,  sin(-np.pi/2),   0],
        			  [              0,               1,    		  0,   0],
        			  [ -sin(-np.pi/2),               0,  cos(-np.pi/2),   0],
        			  [              0, 		      0,    		  0,   1]])

        R_corr = simplify(R_z * R_y) #Total rotation

        T_Total = simplify(T0_G * R_corr) #Total Homogeneous transform
        
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
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
     
            ### Your IK code here 
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
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
