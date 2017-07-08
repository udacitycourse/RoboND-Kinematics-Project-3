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
from sympy import symbols,cos,sin,pi,simplify,sqrt,atan2
from sympy.matrices import Matrix
import numpy as np


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print ("No valid poses received")
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        
        
        # Define DH param symbols
            
        # Joint variables (thetas)
        q1,q2,q3,q4,q5,q6,q7 = symbols("q1:8")

        # Offsets
        d1,d2,d3,d4,d5,d6,d7 = symbols("d1:8")
        
        # Link length
        a0,a1,a2,a3,a4,a5,a6 = symbols("a0:7")
        
        # Joint angle
        alpha0,alpha1,alpha2,alpha3,alpha4,alpha5,alpha6 = symbols("alpha0:7")
            
        # Modified DH params
        #q2 = q2 - pi / 2

        
        # Define Modified DH Transformation matrix
        s = { alpha0 : 0,     a0 : 0,     d1 : 0.75,                # 0.33 + 0.42 = 0.75
              alpha1 : -pi/2, a1 : 0.35,  d2 : 0,     q2 : q2-pi/2,
              alpha2 : 0,     a2 : 1.25,  d3 : 0,            
              alpha3 : -pi/2, a3 : -0.54, d4 : 1.5,                 # 0.96 + 0.54 = 1.5
              alpha4 : pi/2,  a4 : 0,     d5 : 0,     
              alpha5 : -pi/2, a5 : 0,     d6 : 0,     
              alpha6 : -pi/2, a6 : 0,     d7 : 0.303, q7 : 0   }    # 0.193 + 0.11 = 0.303
        
        
        


        # Create individual transformation matrices
        
        #### Homogeneous Transforms
        T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
                       [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
                       [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
                       [                   0,                   0,            0,               1]])
        T0_1 = T0_1.subs(s)
        

        T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
                       [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
                       [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
                       [                   0,                   0,            0,               1]])
        T1_2 = T1_2.subs(s)
        
        T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
                       [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
                       [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
                       [                   0,                   0,            0,               1]])
        T2_3 = T2_3.subs(s)
        
        T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3],
                       [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
                       [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
                       [                   0,                   0,            0,               1]])
        T3_4 = T3_4.subs(s)
        
        T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
                       [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
                       [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
                       [                   0,                   0,            0,               1]])
        T4_5 = T4_5.subs(s)
        
        T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5],
                       [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
                       [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
                       [                   0,                   0,            0,               1]])
        T5_6 = T5_6.subs(s)

        T6_G = Matrix([[             cos(q7),            -sin(q4),            0,              a6],
                       [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
                       [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
                       [                   0,                   0,            0,               1]])
        T6_G = T6_G.subs(s)
        
        
        # Base link to next till gripper
        
        T0_2 = simplify(T0_1 * T1_2)
        T0_3 = simplify(T0_2 * T2_3)
        '''
        T0_4 = simplify(T0_3 * T3_4)
        T0_5 = simplify(T0_4 * T4_5)            
        T0_6 = simplify(T0_5 * T5_6)
        T0_G = simplify(T0_6 * T6_G)
        '''
        # difference in pi from simpy and np.pi ???
        

                       
        T0_2 = T0_2 * Matrix([0, 0, 0, 1])
                       
        
        # Rotate Z and Y to aling with gripper

        R_z = Matrix([[ cos(np.pi),    -sin(np.pi),   0],
                      [ sin(np.pi),     cos(np.pi),   0],
                      [          0,              0,   1]])
        
        R_y = Matrix([[ cos(-np.pi/2),    0,   sin(-np.pi/2)],
                      [             0,    1,               0],
                      [-sin(-np.pi/2),    0,   cos(-np.pi/2)]])            
        
        R_rot = simplify(R_z * R_y)
        
        
        # Result
        #R_res = simplify(T0_G[0:3,0:3] * R_rot)
        #print(R_res)
        
        
        # Initialize variables

        theta1, theta2, theta3, theta4, theta5, theta6 = 0, 0, 0, 0, 0, 0
        save_theta = (0, 0, 0, 0, 0, 0)
        
        R0_3 = T0_3[0:3,0:3]
        
        R0_3_inv = simplify(R0_3 ** -1)
        
        
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
     
            # Calculate joint angles using Geometric IK method
             
            # Roll around X
            R_roll = Matrix([[ 1,         0,          0,],
                             [ 0, cos(roll), -sin(roll),],
                             [ 0, sin(roll),  cos(roll),]])
            # Pitch around Y
            R_pitch = Matrix([[ cos(pitch),        0,  sin(pitch)],
                              [          0,        1,           0],
                              [-sin(pitch),        0,  cos(pitch)]])
            # Yaw around Z
            R_yaw = Matrix([[ cos(yaw), -sin(yaw),     0],
                            [ sin(yaw),  cos(yaw),     0],
                            [        0,         0,     1]])
            # Full rotation from Roll, Pitch and Yaw
            R0_6 = simplify(R_roll * R_pitch * R_yaw)
            
            # Aling with gripper
            R0_G = R0_6 * R_rot
            
            
            
            # Find wrist center
            WC_0 = Matrix([[px], [py], [pz]]) - 0.303 * R0_G * Matrix([[0], [0], [1]])
            

            # theta 1 with a wrist center
            theta1 = np.float64(atan2(WC_0[1], WC_0[0]))
            
            
            # Calculate theta2
            I2_0 = T0_2.evalf(subs={q1: theta1})            
            WC_1 = WC_0 - I2_0[0:3,:]
            Len35 = sqrt(a3**2 + d4**2)
            P25 = sqrt(np.sum(np.square(WC_1)))
            C523 = (-Len35**2 + a2**2 + P25**2) / (2*a2 * P25)
            theta2 = np.float64((pi/2 - (atan2(sqrt(1 - C523**2), C523) + 
	                                 atan2(WC_1[2], sqrt(WC_1[0]**2 + WC_1[1]**2)))).subs(s))

            # Calculate theta3
            C235 = (np.sum(np.square(WC_1)) - a2**2 - Len35**2) / (2*a2*Len35)
            theta3 = np.float64((atan2(sqrt(1-C235**2), C235) + atan2(a3, d4) - pi/2).subs(s))

            # step 3 - calculate theta 4-6
            R0_3_num = R0_3_inv.evalf(subs={q1:theta1, q2:theta2, q3:theta3})
            R3_6_sym = R0_3_num * R0_G
            R3_6 = R3_6_sym.evalf(subs={q1:theta1, q2:theta2, q3:theta3})
            
            
            # based on(pseudo code on page 5)
            # http://thomasbeatty.com/MATH%20PAGES/ARCHIVES%20-%20NOTES/Applied%20Math/euler%20angles.pdf
            # the way that discussed in the lesson made arm spil like crazy and it took was knocking down
            # objects instead of picking them
            
            if np.abs(R3_6[1,2]) is not 1:
                theta5 = np.float64(atan2( sqrt(R3_6[0,2]**2 + R3_6[2,2]**2), R3_6[1,2] ))
                if sin(theta5) < 0:
                    theta4 = np.float64(atan2(-R3_6[2,2], R3_6[0,2]))
                    theta6 = np.float64(atan2(R3_6[1,1], -R3_6[1,0]))
                else:
                    theta4 = np.float64(atan2(R3_6[2,2], -R3_6[0,2]))
                    theta6 = np.float64(atan2(-R3_6[1,1], R3_6[1,0]))
            else:
                theta6 = prev_angles[5]
                if R3_6[1,2] == 1:
                    theta5 = np.float64(0)
                    theta4 = np.float64(-theta6 + atan2(-R3_6[0,1], -R3_6[2,1]))
                else:
                    theta5 = np.float64(0)
                    theta4 = np.float64(theta6 - atan2(R3_6[0,1], -R3_6[2,1]))
            
            save_theta = (theta1, theta2, theta3, theta4, theta5, theta6)
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
    print ("Ready to receive an IK request")
    rospy.spin()

if __name__ == "__main__":
    IK_server()
