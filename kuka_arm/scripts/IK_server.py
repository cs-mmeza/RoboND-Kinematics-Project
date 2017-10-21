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
	#
	#
	# Extract rotation matrices from the transformation matrices
	#
	#
        ###

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
