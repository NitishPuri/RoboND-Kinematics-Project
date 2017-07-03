#!/usr/bin/env/python

import numpy as np
import rospy
import tf
from numpy import array
from sympy import symbols, cos, sin, pi, simplify, sqrt, atan2
from sympy.matrices import Matrix
import timeit

def rot_x(q):
    R_x = Matrix([[       1,       0,       0 ],
                  [       0,  cos(q), -sin(q) ],
                  [       0,  sin(q),  cos(q) ]])
    return R_x

def rot_y(q):
    R_y = Matrix([[  cos(q),       0,  sin(q) ],
                  [       0,       1,       0 ],
                  [ -sin(q),       0,  cos(q) ]])
    return R_y

def rot_z(q):
    R_z = Matrix([[  cos(q), -sin(q),       0 ],
                  [  sin(q),  cos(q),       0 ],
                  [       0,       0,       1 ]])
    return R_z



### Create symbols for joint  variables
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')  # theta_i
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')  # link_offset_i
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')  # link_length_i
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')  # link_twist_i


### Kuka KR210 ###
# DH Parameters
s = {alpha0:     0, a0:      0, d1:  0.75,
     alpha1: -pi/2, a1:   0.35, d2:     0,  q2: q2-pi/2,
     alpha2:     0, a2:   1.25, d3:     0,
     alpha3: -pi/2, a3: -0.054, d4:  1.50,
     alpha4:  pi/2, a4:      0, d5:     0,
     alpha5: -pi/2, a5:      0, d6:     0,
     alpha6:     0, a6:      0, d7: 0.303,  q7: 0 }

def createMatrix(alpha, a, q, d):
    mat =  Matrix([[             cos(q),            -sin(q),            0,              a],
                   [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                   [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                   [                   0,                   0,            0,               1]])
    return mat


#### Homogeneous Transforms
# base_link to link1    
T0_1 = createMatrix(alpha0,a0, q1, d1)
T0_1 = T0_1.subs(s)

T1_2 = createMatrix(alpha1, a1, q2, d2)
T1_2 = T1_2.subs(s)

T2_3 = createMatrix(alpha2, a2, q3, d3)
T2_3 = T2_3.subs(s)

T3_4 = createMatrix(alpha3, a3, q4, d4)
T3_4 = T3_4.subs(s)

T4_5 = createMatrix(alpha4, a4, q5, d5)
T4_5 = T4_5.subs(s)

T5_6 = createMatrix(alpha5, a5, q6, d6)
T5_6 = T5_6.subs(s)

T6_G = createMatrix(alpha6, a6, q7, d7)
T6_G = T6_G.subs(s)

# Composition of Homogenous Transforms
T0_2 = simplify(T0_1 * T1_2) # base_link to link 2
T0_3 = simplify(T0_2 * T2_3) # base_link to link 3
T0_4 = simplify(T0_3 * T3_4) # base_link to link 3
T0_5 = simplify(T0_4 * T4_5) # base_link to link 3
T0_6 = simplify(T0_5 * T5_6) # base_link to link 3
T0_G = simplify(T0_6 * T6_G) # base_link to link 3

# Correction Needed to account for orientation difference between definition
# of gripper_link in URDF versus DH Convention
R_corr = simplify(rot_z(np.pi) * rot_y(-np.pi/2))
R_corr = R_corr[0:3, 0:3] # Extract rotation matrix from homogeneous transform

### Numerically evaluate transforms (compare this with output of tf_echo!)
# print("T0_1 = ", T0_1.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5:0, q6:0}))
# print("T0_2 = ", T0_2.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5:0, q6:0}))
# print("T0_3 = ", T0_3.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5:0, q6:0}))
# print("T0_4 = ", T0_4.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5:0, q6:0}))
# print("T0_5 = ", T0_5.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5:0, q6:0}))
# print("T0_6 = ", T0_6.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5:0, q6:0}))
print("T0_G = ", T0_G.evalf(subs={q1:-0.11, q2:-0.72, q3:-0.27, q4:1.17, q5:1.10, q6:1.17}))


# end-effector position and orientation w.r.t base_link
# px, py, pz = 2.15292, 0, 1.9465  # Test1
px, py, pz = -2.9466, -0.006483, 2.0676  # Test2
# px, py, pz = 2.15292, 0, 1.9465  # Test1
# px, py, pz = 2.15292, 0, 1.9465  # Test1
# px, py, pz = 2.15292, 0, 1.9465  # Test1
# px, py, pz = 2.15292, 0, 1.9465  # Test1
# px, py, pz = 2.15292, 0, 1.9465  # Test1
# px, py, pz = 0.585253, 0.186993, 2.97046
# roll, pitch, yaw = 0, 0, 0
# quat = [0, -0.00014835, 0, 1] # Test 1
quat = [-0.25644, -0.53055, 0.77312, 0.2] # Test 2
# quat = [0, -0.00014835, 0, 1] # Test 1
# quat = [0, -0.00014835, 0, 1] # Test 1
# quat = [0, -0.00014835, 0, 1] # Test 1
# quat = [0, -0.00014835, 0, 1] # Test 1
# quat = [0.70787, 0.26361, 0.34246, 0.55870]
pg_0 = ([[px], [py], [pz]])
R0_g = tf.transformations.quaternion_matrix(quat)
# R0_g = tf.transformations.euler_matrix(roll, pitch, yaw)
R0_g = R0_g[0:3, 0:3]  # Extract the rotation matrix from the transformation


def rtod(q):
    return q*180.0/np.pi

# Calculate wrist-center
d_g = s[d7]
z_g = np.dot(R_corr.tolist(), ([[0], [0], [1]]))
rwc_0 = pg_0 - d_g* np.dot(R0_g,z_g)

theta1 = atan2(rwc_0[1], rwc_0[0])
print(theta1.evalf())
pO2_0 = Matrix([[s[a1]], [0], [s[d1]]])
pO2_0 = rot_z(theta1)* pO2_0
print(pO2_0)
pO2towc_0 = rwc_0 - pO2_0
pO2towc_0

A = s[a2]
# Distance between O2 and O5/WC
B = pO2towc_0.norm()
# Distance between O3 and O5/WC = (d4^2 + a3^2) in figure
C = np.sqrt(s[d4]*s[d4] + s[a3]*s[a3]) 

beta_prime = atan2(s[a3], s[d4])

gamma = np.arccos(float((A*A + B*B - C*C)/(2*A*B)))

beta = np.arccos(float((A*A + C*C - B*B)/(2*A*C)))
alpha = np.arccos(float((B*B + C*C - A*A)/(2*B*C)))

# test = rtod(alpha) + rtod(beta) + rtod(gamma)

def closeEnough(a, b):
    if(a - b) <= 0.0001:
        return True
    return False

# np.real_if_close(, 180)
closeEnough(rtod(alpha) + rtod(beta) + rtod(gamma), 180.0)
# closeEnough(rtod(beta) + rtod(beta_prime), 90.0)
# alpha = 
pO2towc_0 = pO2towc_0.normalized()
pO2towc_0 = pO2towc_0.row_insert(3, Matrix([0]))

# z_2prime is the Z2 axis when q1 = theta1, this does not depend upon q2 
z_2prime = T0_2.subs({q1:theta1}).dot([[0], [0], [1], [0]])
# z_2prime = z_2prime.normalized()


z2_rotation =  tf.transformations.rotation_matrix(-gamma, z_2prime)

# rot_piby2 = tf.transformations.rotation_matrix(-np.pi/2, z_2prime)
# testaxis = rot_piby2 * Matrix([[1], [0], [0], [0]]) 
# tf.transformations.quat
# quaternion_about_axis(gamma, z_2prime[0:3])
# tf.transformations.quaternion_from_matrix(z2_rotation)
a_dir = z2_rotation * pO2towc_0
a_dir = a_dir.normalized()


pO3 = pO2_0 + Matrix((A*a_dir).tolist()[0:3])
print(pO3)
c_dir = (rwc_0 - pO3).normalized()

# Compute theta2
X2_prime = T0_2.subs({q1:theta1, q2:0}).dot([[1], [0], [0], [0]])
theta2 = np.arccos(float(np.dot(X2_prime, a_dir[0:4]) ))
print(theta2)
Y3_prime = T0_3.subs({q1:theta1, q2:theta2, q3: 0}).dot([[0], [1], [0], [0]])
theta3 = np.arccos(float(np.dot(Y3_prime[0:3], c_dir[:]))) + beta_prime

R0_3 = T0_3[0:3,0:3]

R3_6 = R0_3.transpose()* Matrix(R0_g)*R_corr.transpose()
R3_6 = R3_6.subs({q1: theta1, q2:theta2, q3: theta3})

R3_6_prime = (T3_4 * T4_5 * T5_6)
R3_6_prime = R3_6_prime[:3,:3]


theta4 = np.arctan(float(-R3_6[8]/R3_6[2])) # +- np.pi
theta6 = np.arctan(float(-R3_6[4]/R3_6[3])) # +- np.pi
theta5 = np.arccos(float(R3_6[5])) # -theta5

# theta5_2 = np.arctan(float((-R3_6[4]/np.sin(theta6))/R3_6[5]))

# Now there are 2*2*2 possible solutions, we find the correct ones that satisfy the complete matrix

# sf_1 = {q4:theta4, q6:theta6, q5: theta5}
# sf_2 = {q4:theta4, q6:theta6, q5: -theta5}
# sf_3 = {q4:theta4, q6:theta6 + np.pi, q5: theta5}
# sf_4 = {q4:theta4, q6:theta6 + np.pi, q5: -theta5}
# sf_5 = {q4:theta4 + np.pi, q6:theta6, q5: theta5}
# sf_6 = {q4:theta4 + np.pi, q6:theta6, q5: -theta5}
# sf_7 = {q4:theta4 + np.pi, q6:theta6 + np.pi, q5: theta5}
# sf_8 = {q4:theta4 + np.pi, q6:theta6 + np.pi, q5: -theta5}

sf_arr = [ {q4:theta4, q6:theta6, q5: theta5},
       {q4:theta4, q6:theta6, q5: -theta5},
       {q4:theta4, q6:theta6 + np.pi, q5: theta5},
       {q4:theta4, q6:theta6 + np.pi, q5: -theta5},
       {q4:theta4 + np.pi, q6:theta6, q5: theta5},
       {q4:theta4 + np.pi, q6:theta6, q5: -theta5},
       {q4:theta4 + np.pi, q6:theta6 + np.pi, q5: theta5},
       {q4:theta4 + np.pi, q6:theta6 + np.pi, q5: -theta5} ]


# R3_6.is


solution = {}
# for b in l
for sf in sf_arr:
    R3_6_prime_val = R3_6_prime.subs(sf)
    print(sf)
    # np.isclose(R3_6_prime_val.tolist(), R3_6.tolist())
    if (R3_6 - R3_6_prime_val).evalf(10, chop = True).is_zero :
        print("Found solution")
        solution = sf
        break;


theta4 = sf[q4]
theta5 = sf[q5]
theta6 = sf[q6]

print(theta1, theta2, theta3, theta4, theta5, theta6)
# R3_6_prime_1 = R3_6_prime.subs(sf[1 ])
# R3_6_prime_2 = R3_6_prime.subs({q4:theta4, q6:theta6, q5: -theta5})
# R3_6_prime_3 = R3_6_prime.subs({q4:theta4, q6:theta6 + np.pi, q5: theta5})
# R3_6_prime_4 = R3_6_prime.subs({q4:theta4, q6:theta6 + np.pi, q5: -theta5})
# R3_6_prime_5 = R3_6_prime.subs({q4:theta4 + np.pi, q6:theta6, q5: theta5})
# R3_6_prime_6 = R3_6_prime.subs({q4:theta4 + np.pi, q6:theta6, q5: -theta5})
# R3_6_prime_7 = R3_6_prime.subs({q4:theta4 + np.pi, q6:theta6 + np.pi, q5: theta5})
# R3_6_prime_8 = R3_6_prime.subs({q4:theta4 + np.pi, q6:theta6 + np.pi, q5: -theta5})



# d = 0.303 # 0.193 + 0.11
# rwc_0 = pg_0 - d* np.dot(R0_g,([[1], [0], [0]]))
# rwc_0

# theta1 =  atan2(rwc_0[1], rwc_0[0])

# position of O2 w.r.t. base_link 
# pO2_0 = [[s[a1]], [0], [s[d1]]]

# # pO2_0 = rot_z(theta1).dot(pO2_0)
# pO2_0 = np.dot(rot_z(theta1).tolist(), pO2_0)
# # pO2_0 = T0_2.subs({q1:theta1}).dot([[0], [0], [0], [1]])

# # Distance between O2 and O3 = a2(in figure)
# A = s[a2]
# # Distance between O2 and O5/WC
# B = (pO2_0 - rwc_0).norm()
# # Distance between O3 and O5/WC = (d4^2 + a3^2) in figure
# C = np.sqrt(s[d4]*s[d4] + s[a3]*s[a3]) 


# gamma = 


# position of O3 w.r.t. base_link in zero configuration
pO3_0 = [[0.35], [0], [2]]  # [[a1], [0], [d1 + a2]]
# pO3_0 = rot_z(theta1).dot()

# position of O5 w.r.t. base_link in zero configuration
# pO5_0 = [[0.35], [0], [0.75]]
# pO5_0 = rot_z(theta1).dot(pO2_0)




# Total Homogeneous Transform Between Base_link and Gripper_link with
# Orientation Correction Applied
T_total = simplify(T0_G * R_corr)


# end = timeit.timeit()
# print("Total transform ,..   ", (end - start))
print("T_total = ", T_total.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5:0, q6:0}))
# start = end

from IK_server import * 
solver = KukaIKSolver()
# req = DummyReq([2.1529, 0, 1.9465], [0, -0.00014835, 0, 1])
# req = DummyReq([1.7441, 1.26208, 1.94653], [4.57081e-05, -0.000141136, 0.308104, 0.951353])
# req = DummyReq([-2.63197, 0.669588, 1.98236], [0.685169, -0.575695, 0.444723, -0.0366073])
req = solver.performFK([0, 0, 0, 0, 0, 0])
req = solver.performFK([3.21, 1.16, -1.81, -4.04, -1.07, -3.72])
# req = solver.performFK([3.21, 1.16, -1.81, -4.04, -1.07, -3.72])
3.21, 1.16, -1.81, -4.04, -1.07, -3.72
req = solver.generateRandomReq()
m =solver.T0_2.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5:0, q6:0})
tf.transformations.euler_from_matrix(m.tolist())
tf.transformations.euler_matrix(-np.pi/2, -np.pi/2, 0)
req.printReq()
solver.handle_calculate_IK2(req)
(2.917867671547474, 0.009862711191248374, -2.7209286539888358, 2.9404270337063627, 0.4970054939892705, 3.6537983704252612)