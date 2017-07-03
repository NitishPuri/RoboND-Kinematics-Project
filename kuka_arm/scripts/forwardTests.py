#!/usr/bin/env/python

import numpy as np
from numpy import array
from sympy import symbols, cos, sin, pi, simplify, sqrt, atan2
from sympy.matrices import Matrix
import timeit

print("Starting up,...")
start = timeit.timeit()

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

# end = timeit.timeit()
# print("Symbols and dictioinary created,..   ", (end - start))
# start = end

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

# end = timeit.timeit()
# print("T0_1 = ", T0_1)
# print("T0_1 created in ,..   ", (end - start))
# start = end

T1_2 = createMatrix(alpha1, a1, q2, d2)
T1_2 = T1_2.subs(s)

# end = timeit.timeit()
# print("T1_2 = ", T1_2)
# print("T1_2 created in ,..   ", (end - start))
# start = end

T2_3 = createMatrix(alpha2, a2, q3, d3)
T2_3 = T2_3.subs(s)

# end = timeit.timeit()
# print("T2_3 = ", T2_3)
# print("T2_3 created in ,..   ", (end - start))
# start = end

T3_4 = createMatrix(alpha3, a3, q4, d4)
T3_4 = T3_4.subs(s)

# end = timeit.timeit()
# print("T3_4 = ", T3_4)
# print("T3_4 created in ,..   ", (end - start))
# start = end

T4_5 = createMatrix(alpha4, a4, q5, d5)
T4_5 = T4_5.subs(s)

# end = timeit.timeit()
# print("T4_5 = ", T4_5)
# print("T4_5 created in ,..   ", (end - start))
# start = end

T5_6 = createMatrix(alpha5, a5, q6, d6)
T5_6 = T5_6.subs(s)

# end = timeit.timeit()
# print("T5_6 = ", T5_6)
# print("T5_6 created in ,..   ", (end - start))
# start = end

T6_G = createMatrix(alpha6, a6, q7, d7)
T6_G = T6_G.subs(s)

# end = timeit.timeit()
# print("T6_G = ", T6_G)
# print("T6_G created in ,..   ", (end - start))
# start = end


# Composition of Homogenous Transforms
T0_2 = simplify(T0_1 * T1_2) # base_link to link 2
T0_3 = simplify(T0_2 * T2_3) # base_link to link 3
T0_4 = simplify(T0_3 * T3_4) # base_link to link 3
T0_5 = simplify(T0_4 * T4_5) # base_link to link 3
T0_6 = simplify(T0_5 * T5_6) # base_link to link 3
T0_G = simplify(T0_6 * T6_G) # base_link to link 3

end = timeit.timeit()
print("Composition og Homogeneous transforms ,..   ", (end - start))
# print("T0_1 = ", T0_1)
# print("T0_2 = ", T0_2)
# print("T0_3 = ", T0_3)
# print("T0_4 = ", T0_4)
# print("T0_5 = ", T0_5)
# print("T0_6 = ", T0_6)
# print("T0_G = ", T0_G)
start = end


# Correction Needed to account for orientation difference between definition
# of gripper_link in URDF versus DH Convention
R_z = Matrix([[     cos(np.pi), -sin(np.pi),             0,  0],
              [     sin(np.pi),  cos(np.pi),             0,  0],
              [              0,           0,             1,  0],
              [              0,           0,             0,  1]])  
R_y = Matrix([[  cos(-np.pi/2),           0, sin(-np.pi/2),  0],
              [              0,           1,             0,  0],
              [ -sin(-np.pi/2),           0, cos(-np.pi/2),  0],
              [              0,           0,             0,  1]])  
R_corr = simplify(R_z * R_y)

# end = timeit.timeit()
# print("Matrices for rotation correction of gripper_link calculated ,..   ", (end - start))
# print("R_corr = ", R_corr)
# start = end

### Numerically evaluate transforms (compare this with output of tf_echo!)
# print("T0_1 = ", T0_1.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5:0, q6:0}))
# print("T0_2 = ", T0_2.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5:0, q6:0}))
# print("T0_3 = ", T0_3.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5:0, q6:0}))
# print("T0_4 = ", T0_4.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5:0, q6:0}))
# print("T0_5 = ", T0_5.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5:0, q6:0}))
print("T0_6 = ", T0_6.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5:0, q6:0}))
print("T0_G = ", T0_G.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5:0, q6:0}))

end = timeit.timeit()
print("Numerical Evaluation of transforms done in ,..   ", (end - start))
start = end


# Total Homogeneous Transform Between Base_link and Gripper_link with
# Orientation Correction Applied
T_total = simplify(T0_G * R_corr)


# end = timeit.timeit()
# print("Total transform ,..   ", (end - start))
print("T_total = ", T_total.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5:0, q6:0}))
# start = end

import matplotlib.pyplot as plt
import numpy as np

x_s = xrange(0, 10)
y_s = [np.random.rand()*5 for x in x_s]
fig, ax = plt.subplots()
ax.bar(x_s, y_s)
plt.show()