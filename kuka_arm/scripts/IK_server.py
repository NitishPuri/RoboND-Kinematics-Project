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
import numpy as np
import matplotlib.pyplot as plt
import time


printLogs = True

# Rotation Matrix about X
def rot_x(q):
    R_x = Matrix([[       1,       0,       0 ],
                  [       0,  cos(q), -sin(q) ],
                  [       0,  sin(q),  cos(q) ]])
    return R_x

# Rotation Matrix about Y
def rot_y(q):
    R_y = Matrix([[  cos(q),       0,  sin(q) ],
                  [       0,       1,       0 ],
                  [ -sin(q),       0,  cos(q) ]])
    return R_y

# Rotation Matrix about Z
def rot_z(q):
    R_z = Matrix([[  cos(q), -sin(q),       0 ],
                  [  sin(q),  cos(q),       0 ],
                  [       0,       0,       1 ]])
    return R_z

# Predicate to check closeness of two numbers
def closeEnough(a, b, tol = 0.0001):
    if(a - b) <= tol:
        return True
    return False

# Create transformation matrix between two links 
# according to Modified DH convention with given parameters  
def createMatrix(alpha, a, q, d):
    mat =  Matrix([[            cos(q),           -sin(q),           0,             a],
                   [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                   [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                   [                 0,                 0,           0,             1]])

    return mat


def generateRandomInRange(low, high):
    return low + np.random.rand()*(high - low)

# Radians to Degree
def rtod(q):
    return q*180.0/np.pi

# Degree to Radians
def dtor(q):
    return q*np.pi/180.0

def checkInRange(val, low, high, name):
    if((val >= low) & (val <= high)):
        return True
    else:
        print ("******Out of range {}[{}], ({}, {})******".format(name, val, low, high))
        return False;

def debugLog(log):
    # print (PrintLogs)
    if (printLogs):
        print(log)
    pass


def createPlot(errors):
    plt.close()
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ax.bar(range(0, len(errors)), errors)
    ax.set_xlabel("poses")
    ax.set_ylabel("errors")
    ax.set_ylim(0, 10)
    fig.savefig("plots/{} errors.png".format(time.ctime()))

def saveErrors(errors):
    np.savetxt("plots/errors {}".format(time.ctime()), errors)




########################################################################################
# Defining some global variables to allow easy access without writing self.xxx all the time
########################################################################################
# #### Step 1 
# # Define DH param symbols
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')  # link_offset_i
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')  # link_length_i
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')  # link_twist_i

# # Joint angle symbols
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')  # theta_i

# ### Kuka KR210 ###
# # DH Parameters
s = {alpha0:     0, a0:      0, d1:  0.75,
    alpha1: -pi/2, a1:   0.35, d2:     0,  q2: q2-pi/2,
    alpha2:     0, a2:   1.25, d3:     0,
    alpha3: -pi/2, a3: -0.054, d4:  1.50,
    alpha4:  pi/2, a4:      0, d5:     0,
    alpha5: -pi/2, a5:      0, d6:     0,
    alpha6:     0, a6:      0, d7: 0.303,  q7: 0 }
########################################################################################
########################################################################################

class DummyReq(object):
    # poses = [Pose()]
    def __init__(self, pos, orient):
        self.poses = [Pose()]
        self.poses[0].position.x = pos[0]
        self.poses[0].position.y = pos[1]
        self.poses[0].position.z = pos[2]
        self.poses[0].orientation.x = orient[0]
        self.poses[0].orientation.y = orient[1]
        self.poses[0].orientation.z = orient[2]
        self.poses[0].orientation.w = orient[3]
    
    def printReq(self):
        print("Pos : ", self.poses[0].position)
        print("Orient : ", self.poses[0].orientation)



# Class to solve IK problem for the Kuka KR210
# This class creates the symbolic transforms only once during initialization
class KukaIKSolver(object):


    def __init__(self):

        # # Define Modified DH Transformation matrix
        # #### Homogeneous Transforms
        # return
        self.T0_1 = createMatrix(alpha0,a0, q1, d1)
        self.T0_1 = self.T0_1.subs(s)

        self.T1_2 = createMatrix(alpha1, a1, q2, d2)
        self.T1_2 = self.T1_2.subs(s)

        self.T2_3 = createMatrix(alpha2, a2, q3, d3)
        self.T2_3 = self.T2_3.subs(s)

        self.T3_4 = createMatrix(alpha3, a3, q4, d4)
        self.T3_4 = self.T3_4.subs(s)

        self.T4_5 = createMatrix(alpha4, a4, q5, d5)
        self.T4_5 = self.T4_5.subs(s)

        self.T5_6 = createMatrix(alpha5, a5, q6, d6)
        self.T5_6 = self.T5_6.subs(s)

        self.T6_G = createMatrix(alpha6, a6, q7, d7)
        self.T6_G = self.T6_G.subs(s)

        # # Composition of Homogenous Transforms
        self.T0_2 = simplify(self.T0_1 * self.T1_2) # base_link to link 2
        self.T0_3 = simplify(self.T0_2 * self.T2_3) # base_link to link 3
        self.T0_4 = simplify(self.T0_3 * self.T3_4) # base_link to link 3
        self.T0_5 = simplify(self.T0_4 * self.T4_5) # base_link to link 3
        self.T0_6 = simplify(self.T0_5 * self.T5_6) # base_link to link 3
        self.T0_G = simplify(self.T0_6 * self.T6_G) # base_link to link 3

        # # Correction Needed to account for orientation difference between definition
        # # of gripper_link in URDF versus DH Convention
        self.R_corr = Matrix(simplify(rot_z(pi) * rot_y(-pi/2)))
        # self.R_corr = self.R_corr[0:3, 0:3] # Extract rotation matrix from homogeneous transform

        # Compute complete transform for End effector
        R_corr2 = self.R_corr.row_insert(3, Matrix([[0, 0, 0]]))
        R_corr2 = R_corr2.col_insert(3, Matrix([0, 0, 0, 1]))
        self.T_total = simplify(self.T0_G * R_corr2)


        # Rotation transform between link 3 and 6, defined symbollically based 
        # on the Modified DH parameters. 
        # This is used to solve for joints 4,5,6 by comparing with computed R3_6
        self.R3_6_prime = (self.T3_4 * self.T4_5 * self.T5_6)
        self.R3_6_prime = self.R3_6_prime[:3,:3]
        # R3_6_prime = 
        # [-s4s6 +c4c5c6,-s4c6 -s6c4c5,-s5c4]
        # [         s5c6,        -s5s6,   c5]
        # [-s4c5c6 -s6c4, s4s6c5 -c4c6, s4s5]
        # Where, s = sin, c =cos, 4,5,6 = q4,q5,q6
        # So that, -s5c6 = -sin(q5)cos(q6)


        # Memoization for theta4 and theta6
        self.old_theta4 = 0
        self.old_theta6 = 0
        pass

    def performFK(self, theta_t):
        theta_s = {q1: theta_t[0], q2: theta_t[1], q3: theta_t[2], q4: theta_t[3], q5: theta_t[4], q6:theta_t[5]}
        # theta_s = {q1: theta_t[0], q}
        debugLog("Values generated by FK on given angles ::::::\n")
        origin = Matrix([[0], [0], [0], [1]])

        T0_2_prime = self.T0_2.evalf(subs=theta_s) 
        p2 = T0_2_prime*origin
        rpy2 = tf.transformations.euler_from_matrix(T0_2_prime.tolist())
        quat2 = tf.transformations.quaternion_from_matrix(T0_2_prime.tolist())
        debugLog("Link 2 position : {}".format(p2.tolist()))

        T0_3_prime = self.T0_3.evalf(subs=theta_s) 
        p3 = T0_3_prime*origin
        rpy3 = tf.transformations.euler_from_matrix(T0_3_prime.tolist())
        quat3 = tf.transformations.quaternion_from_matrix(T0_3_prime.tolist())
        debugLog("Link 3 position : {}".format(p3.tolist()))

        T0_5_prime = self.T0_5.evalf(subs=theta_s) 
        p5 = T0_5_prime*origin
        rpy5 = tf.transformations.euler_from_matrix(T0_5_prime.tolist())
        quat5 = tf.transformations.quaternion_from_matrix(T0_5_prime.tolist())
        debugLog("Link 5/Wrist Center position : {}".format(p5.tolist()))

        T0_G_prime = self.T0_G.evalf(subs=theta_s) 
        pG = T0_G_prime*origin
        rpyG = tf.transformations.euler_from_matrix(T0_G_prime.tolist())
        quatG = tf.transformations.quaternion_from_matrix(T0_G_prime.tolist())
        debugLog("Gripper/End Effector position : {}".format(pG.tolist()))

        # T_total = simplify(self.T0_G * R_corr)
        T_total_prime = self.T_total.evalf(subs=theta_s)
        pFinal = T_total_prime*origin
        rpyFinal = tf.transformations.euler_from_matrix(T_total_prime.tolist())
        quatFinal = tf.transformations.quaternion_from_matrix(T_total_prime.tolist())
        debugLog("EE URDF position : {}".format(pFinal.tolist()))
        debugLog("EE URDF quat : {}".format(quatFinal))

        return DummyReq(pFinal, quatFinal)


    def generateRandomReq(self):
        # Generate random theta values in the given ranges,..
        theta1 = generateRandomInRange(dtor(-185), dtor(185))
        theta2 = generateRandomInRange(dtor(-45), dtor(85))
        theta3 = generateRandomInRange(dtor(-210), dtor(155-90))
        theta4 = generateRandomInRange(dtor(-350), dtor(350))
        theta5 = generateRandomInRange(dtor(-125), dtor(125))
        theta6 = generateRandomInRange(dtor(-350), dtor(350))

        # rand_s = {q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6:theta6}
        theta_t = (theta1, theta2, theta3, theta4, theta5, theta6)
        print("Generated angles :")
        print(theta_t)
        return self.performFK(theta_t)
        


    def handle_calculate_IK2(self, req):
        rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))

        if len(req.poses) < 1: 
            print ("No valid poses received")
            return -1
        else:
            # Initialize service response
            joint_trajectory_list = []
            position_errors = []

            for x in xrange(0, len(req.poses)): 
                # IK code starts here
                joint_trajectory_point = JointTrajectoryPoint()
                
                # Extract end-effector position and orientation from request
                # px,py,pz = end-effector position
                # roll, pitch, yaw = end-effector orientation in euler form
                px = req.poses[x].position.x
                py = req.poses[x].position.y
                pz = req.poses[x].position.z

                (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                    [req.poses[x].orientation.x, req.poses[x].orientation.y,
                        req.poses[x].orientation.z, req.poses[x].orientation.w])


                # end-effector position vector
                pg_0 = Matrix([[px], [py], [pz]])

                # p_quat = end-effector orientation in quaternion form
                p_quat = [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w]

                # p_quat = [req.poses[0].orientation.x, req.poses[0].orientation.y,req.poses[0].orientation.z, req.poses[0].orientation.w]
                # R0_g = end-effector(gripper) rotation transformation(4X4)
                R0_g = tf.transformations.quaternion_matrix(p_quat)
                R0_g = R0_g[0:3, 0:3]  # Extract the rotation matrix from the transformation

                #### Step 2
                # Calculate wrist-center position
                # Displacement of end-effector from wrist-center, 
                d_g = s[d7] # 0.303

                # Apply correction to transform Z_g(Modified DH) into Z_g(URDF)
                z_g = np.dot(self.R_corr.tolist(), ([[0], [0], [1]]))

                # rwc_0 = wrist-center position w.r.t. base_link(O_0)
                rwc_0 = pg_0 - Matrix(d_g* np.dot(R0_g,z_g))
                debugLog("Wrist Center : {}".format(rwc_0.tolist()))


                #### Step 3
                # Calculate joint variables q1, q2, q3 for the calculated wrist-center
                # Experiment,....!!!!

                # Project wrist-center to x_0-y_0 plane and find the angle w.r.t. x_0
                theta1 = atan2(rwc_0[1], rwc_0[0]).evalf()

                # position of O2 w.r.t. base_link in zero configuration
                pO2_0 = Matrix([[s[a1]], [0], [s[d1]]])

                pO2_0 = rot_z(theta1)* pO2_0
                debugLog("Link 2 position : {}".format(pO2_0.tolist()))

                # Consider the triangle (O2, O3, WC)
                # O2 , WC are known
                # O3 = unknown
                pO2towc_0 = rwc_0 - pO2_0
                # Distance between O2 and O3 = a2(in figure)
                A = s[a2]
                # Distance between O2 and O5/WC
                B = pO2towc_0.norm()
                # Distance between O3 and O5/WC = (d4^2 + a3^2) in figure
                C = np.sqrt(s[d4]*s[d4] + s[a3]*s[a3]) 

                # Offset angle between the Y3 axis line(O3, O5)
                beta_prime = atan2(s[a3], s[d4])

                # applying cosine rule `C^2 = A^2 + B^2 -2ABcos(gamma)`
                # angle(O3, O2, O5)
                gamma = np.arccos(float((A*A + B*B - C*C)/(2*A*B)))
                
                # angle(O2, O3, O5)
                beta = np.arccos(float((A*A + C*C - B*B)/(2*A*C)))

                pO2towc_0 = pO2towc_0.normalized()
                pO2towc_0 = pO2towc_0.row_insert(3, Matrix([0]))

                # z_2prime is the Z2 axis when q1 = theta1, this does not depend upon q2 
                z_2prime = self.T0_2.subs({q1:theta1}).dot([[0], [0], [1], [0]])
                # z_2prime = z_2prime.normalized()

                # Rotate pO2towc_0 by gamma along z_2prime
                z2_rotation =  tf.transformations.rotation_matrix(-gamma, z_2prime)
                # quaternion_about_axis(gamma, z_2prime[0:3])
                # tf.transformations.quaternion_from_matrix(z2_rotation)
                a_dir = z2_rotation * pO2towc_0
                a_dir = a_dir.normalized()

                # Compute O3
                pO3 = pO2_0 + Matrix((A*a_dir).tolist()[0:3])
                debugLog("Link 3 position : {}".format(pO3.tolist()))
                c_dir = (rwc_0 - pO3).normalized()

                # Compute theta2
                X2_prime = self.T0_2.subs({q1:theta1, q2:0}).dot([[1], [0], [0], [0]])
                theta2 = np.arccos(float(np.dot(X2_prime, a_dir[0:4]) ))

                Y3_prime = self.T0_3.subs({q1:theta1, q2:theta2, q3: 0}).dot([[0], [1], [0], [0]])
                # theta3 = np.arccos(float(np.dot(Y3_prime[0:3], c_dir[:]))) + beta_prime
                theta3 = ((pi/2 + beta_prime) - beta).evalf()


                #### Step 4
                # Calculate R3_0 ## Calculated in the __init__

                #### Step 5 
                # Calculate R3_6
                R0_3 = self.T0_3[0:3,0:3]
                R3_6 = R0_3.transpose()* Matrix(R0_g)*self.R_corr.transpose()
                R3_6 = R3_6.subs({q1: theta1, q2:theta2, q3: theta3})

                # R3_6_prime = 
                # [-s4s6 +c4c5c6,-s4c6 -s6c4c5,-s5c4]
                # [         s5c6,        -s5s6,   c5]
                # [-s4c5c6 -s6c4, s4s6c5 -c4c6, s4s5]
                theta4 = atan2(R3_6[2,2], -R3_6[0, 2]).evalf() 
                theta6 = atan2(-R3_6[1,1], R3_6[1,0]).evalf() 
                theta5 = atan2(sqrt(R3_6[0, 2]*R3_6[0, 2] + R3_6[2, 2]*R3_6[2, 2]), R3_6[1, 2]).evalf() # -theta5

                checkInRange(rtod(theta1), -185, 185, "theta1")
                checkInRange(rtod(theta2),  -45,  85, "theta2")
                checkInRange(rtod(theta3), -210,  65, "theta3")
                checkInRange(rtod(theta4), -350, 350, "theta4")
                checkInRange(rtod(theta5), -125, 125, "theta5")
                checkInRange(rtod(theta6), -350, 350, "theta6")

                def reduceEffectiveRange(theta, name):
                    if(theta < -pi ):
                        print ("***reduceEffectiveRange***")
                        print("{} = {}, being changed to {}".format(name, theta, (theta + 2*pi)))
                        return (theta + 2*pi)
                    elif(theta > pi):
                        print ("***reduceEffectiveRange***")
                        print("{} = {}, being changed to {}".format(name, theta, (theta - 2*pi)))
                        return (theta - 2*pi)
                    else:
                        return theta

                def angleCorrection(theta, old_theta):
                    d = theta - old_theta
                    if d > np.pi:
                        return theta - 2*np.pi
                    elif d < -np.pi:
                        return theta + 2*np.pi
                    else:
                        return theta


                theta4 = reduceEffectiveRange(theta4, "theta4")
                theta6 = reduceEffectiveRange(theta6, "theta6")

                theta4 = angleCorrection(theta4, self.old_theta4)
                theta6 = angleCorrection(theta6, self.old_theta6)

                self.old_theta4 = theta4
                self.old_theta6 = theta6


                # print("Performing FK on the computed result for verification")
                dummmyReq =  self.performFK([theta1, theta2, theta3, theta4, theta5, theta6])
                debugLog("EE position received FK: {}".format(pg_0.tolist()))
                pos_FK = Matrix([[dummmyReq.poses[0].position.x], 
                    [dummmyReq.poses[0].position.y], [dummmyReq.poses[0].position.z]])
                debugLog("  EE position after FK : {}".format(pos_FK.tolist()))
                error = (pg_0 - pos_FK).norm()
                debugLog("              EE Error : {}".format(error))
                position_errors.append(error)


                # sf_arr = [ {q4:theta4, q6:theta6, q5: theta5},
                #         {q4:theta4, q6:theta6, q5: -theta5},
                #         {q4:theta4, q6:theta6 + pi, q5: theta5},
                #         {q4:theta4, q6:theta6 + pi, q5: -theta5},
                #         {q4:theta4 + pi, q6:theta6, q5: theta5},
                #         {q4:theta4 + pi, q6:theta6, q5: -theta5},
                #         {q4:theta4 + pi, q6:theta6 + pi, q5: theta5},
                #         {q4:theta4 + pi, q6:theta6 + pi, q5: -theta5} ]

                # solution = sf_arr[0]

                # for sf in sf_arr:
                #     R3_6_prime_val = self.R3_6_prime.subs(sf)
                #     # print(sf)
                #     # np.isclose(R3_6_prime_val.tolist(), R3_6.tolist())
                #     if (R3_6 - R3_6_prime_val).evalf(10, chop = True).is_zero :
                #         # print("Found solution")
                #         solution = sf
                #         break;

                # theta4 = solution[q4]
                # theta5 = solution[q5]
                # theta6 = solution[q6]

                
                # Populate response for the IK request
                joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
                # rospy.loginfo("req.poses[x].position = {}".format([px, py, pz]))
                rospy.loginfo("joint_trajectory_point.positions = {}".format([theta1, theta2, theta3, theta4, theta5, theta6]))

                joint_trajectory_list.append(joint_trajectory_point)


            # Create a plot showing the error in EE position for the given request.
            if(printLogs == False):
                # createPlot(position_errors)
                # print(position_errors)
                saveErrors(position_errors)
                pass

            rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
            return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service

    rospy.init_node('IK_server')

    print("Initializing Solver,.....")

    # Setting debug log to false during simulation
    global printLogs
    printLogs = False

    solver = KukaIKSolver()
    print("Solver Initialized,.....")
    # solver.handle_calculate_IK2()

    # solver.
    s = rospy.Service('calculate_ik', CalculateIK, solver.handle_calculate_IK2)
    print("Ready to receive an IK request")
    # handle_calculate_IK()
    rospy.spin()

if __name__ == "__main__":
    IK_server()
