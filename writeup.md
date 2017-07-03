## Project: Kinematics Pick & Place

[//]: # (Image References)

<!--[image1]: ./misc_images/misc1.png-->
[annotated_image]: ./misc_images/annotated_image.png
[urdf_annotation]: ./misc_images/urdf_annotation.png
[dh-transform-matrix]: ./misc_images/dh-transform-matrix.png
[dh-transform]: ./misc_images/dh-transform.png
[dh-convention]: ./misc_images/dh-convention.png
[total-transform-corretion]: ./misc_images/total-transform-corretion.png


### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

We can refer to the following images for reference.   
![Annotated image][annotated_image]
**Fig. 1** : Shows the link frames(coordinate systems) choosen according to Modified DH convention. `O(i)` is the origin for link i frame, and `X(i)`, `Z(i)` are the X and Z axis correspondingly, and Z represents the axis of rotation(translation in case of prismatic joints). Since we are using a right handed coordinate system, `Y(i)` can be calculated accordingly.   


![URDF Image][urdf_annotation]
**Fig. 2** : Shows the reference frames as specified in the URDF file. Here, `Ojointi` represents ith frame origin(also represented by a black triangle). It also shows various distance between these joint positions as specified in URDF. We will be refering to these values in our description for DH parameter table.

 Here's the complete `Modified DH Parameter` table, followed by a step by step explaination of all
 the terms.

Link(i) | alpha(i-1) | a(i-1) | d(i) | q(i) 
--- | --- | --- | --- | ---
1 | 0 | 0 | 0.75 | q1
2 | -pi/2 | 0.35 | 0 | q2 - pi/2
3 | 0 | 1.25 | 0 | q3
4 | -p1/2 | -0.054 | 1.50 | q4
5 | pi/2 | 0 | 0 | q5
6 | -pi/2 | 0 | 0 | q6
7(G) | 0 | 0 | 0.303 | 0

Here,   
`alpha(i-1)` : Angle between Z(i-1) and Z(i) measured along X(i-1)   
`a(i-1)` : Link length, distance between Z(i-1) and Z(i), measured along X(i-1)   
`d(i)`  : Link offset, distance between X(i-1) and X(i), measured along Z(i-1), variable in prismatic joints(there are no prismatic joints in the given problem)   
`q(i)` : Joint angle, Angle between X(i-1), X(i) measured along Z(i), variable in revolute joints.   
Also,   
`Link 0` represents the base link, and   
`Link 7` represents the gripper link, which is fixed. It contains left and right grippers(not controlled by IK code.)   

**Link 1 :** `Z0`(0 0 1) is *collinear* to `Z1`(0 0 1), `alpha0 = 0`, `a0 = 0`, `d1 = 0.33(joint1.z) + 0.42(joint2.z) = 0.75`, and `q1` is *unknown*.   
**Link 2 :** `Z1`(0 0 1) is *perpendicular* to `Z2`(0 1 0), so, `alpha1 = -pi/2`, `a1 = 0.35(joint2.x)`, and `d2 = 0` since `X1` intersects `X2` at `O2`. Also, we can see that when joint2 is in *zero* configuration, there is an offset of `-pi/2` from `X1` to `X2`, measured along `Z2`. So, we also need to substitute `q2` with `q2 - pi/2` in the parameter table.   
**Link 3 :**, since `Z2`(0 1 0) is *parallel* to `Z3`(0 1 0), `alpha2 = 0`, `a2 = 1.25(joint3.z)` along `X2`. Also, `X2` and `X3` are collinear, so `d3 = 0`.   
**Link 4 :**, `Z3`(0 1 0) and `Z4`(1 0 0) are *perpendicular*, so `alpha3 = -pi/2` and `a3 = -0.054(joint4.z)`, and `d4 = 0.96(joint4.x) + 0.54(joint5.x) = 1.50`.   
*Note:* We have choosen O4, O5 and O6 to be co-incident with the Wrist Center(WC). This helps in separating the IK problem into computation of the Wrist Center and then Wrist Orientation.   
**Link 5 :**, `Z4`(1 0 0) and `Z5`(0 1 0) are *perpendicular* and *intersect* at `O5`, so `alpha4 = pi/2` and `a4 = 0`. Also, `d5 = 0`, since `X4` and `X4` are *collinear*.   
**Link 6 :**, `Z5`(0 1 0) and `Z6`(1, 0, 0) are *perpendicular* and *intersect* at `O5`, so `alpha5 = -pi/2` and `a5 = 0`, `d6 = 0`, since `X5` and `X6` are *collinear*.   
**Link 7(Gripper Link) :**, this is a fixed link, with a translation along `Z6`. So, `Z6` and `Zg` are *collinear*, so `alpha6 = 0`, `a6 = 0` and `d6 = 0.193(joint6.x) + 0.11(gripper_joint.x)`. Also, since this is fixed(w.r.t link 6), `q7 = 0`.    


#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The following figure represents frame assignment between two links in the Modified DH convention.
![DH Convention][dh-convention]   

From the above image it is clear thet the total transform between `link(i-1)` and `link(i)` can be thought of as a *rotation* by `alpha(i-1)` along `X(i-1)`, *translation* by `a(i-1)` along `X(i-1)`, *rotation* by `q(i)` along `Z(i)`, and finally *translation* by `d(i)` along `Z(i)`. That is,   

![DH Transform][dh-transform]   

Which, when expanded analytically turns out to be,   

![Modified DH Transformation matrix][dh-transform-matrix]   
Where, `cx` represents `cos(x)` and `sx` represents `sin(x)`.   

The same can actually be derived programatically if we combine the individual rotation and translation matrices specified above.

Next, we compute all the transformation matrices between consecutive links. This can be done programatically. Since individual trnasformation between consecutive links only depend on four identifiers, i.e. `alpha(i-1)`, `a(i-1)`, `q(i)` and `d(i)`, we define the following function,...
*Note :* We use `sympy` library for symbollic computation, simplication and substitution.
```python
# Create transformation matrix between two links 
# according to Modified DH convention with given parameters  
def createMatrix(alpha, a, q, d):
    mat =  Matrix([[            cos(q),           -sin(q),           0,             a],
                   [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                   [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                   [                 0,                 0,           0,             1]])

    return mat
```   

Also, we define some symbols for computation and a python dictionary that represents the DH parameter table for our problem.   
```python
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
```

Now, individual matrices can be computed like,   
```python
>>> T0_1 = createMatrix(alpha0,a0, q1, d1).subs(s)
Matrix([
[cos(q1), -sin(q1), 0,    0],
[sin(q1),  cos(q1), 0,    0],
[      0,        0, 1, 0.75],
[      0,        0, 0,    1]])

>>> T1_2 = createMatrix(alpha1,a1, q2, d2).subs(s)
Matrix([
[sin(q2),  cos(q2), 0, 0.35],
[      0,        0, 1,    0],
[cos(q2), -sin(q2), 0,    0],
[      0,        0, 0,    1]])

>>> T2_3 = createMatrix(alpha2,a2, q3, d3).subs(s)
Matrix([
[cos(q3), -sin(q3), 0, 1.25],
[sin(q3),  cos(q3), 0,    0],
[      0,        0, 1,    0],
[      0,        0, 0,    1]])

>>> T3_4 = createMatrix(alpha3,a3, q4, d4).subs(s)
Matrix([
[ cos(q4), -sin(q4), 0, -0.054],
[       0,        0, 1,    1.5],
[-sin(q4), -cos(q4), 0,      0],
[       0,        0, 0,      1]])

>>> T4_5 = createMatrix(alpha4,a4, q5, d5).subs(s)
Matrix([
[cos(q5), -sin(q5),  0, 0],
[      0,        0, -1, 0],
[sin(q5),  cos(q5),  0, 0],
[      0,        0,  0, 1]])

>>> T5_6 = createMatrix(alpha5,a5, q6, d6).subs(s)
Matrix([
[ cos(q6), -sin(q6), 0, 0],
[       0,        0, 1, 0],
[-sin(q6), -cos(q6), 0, 0],
[       0,        0, 0, 1]])

>>> T6_G = createMatrix(alpha6,a6, q7, d7).subs(s)
Matrix([
[1, 0, 0,     0],
[0, 1, 0,     0],
[0, 0, 1, 0.303],
[0, 0, 0,     1]])
```

Also, total homogenous transform *base_link* to *gripper_link* can be computed by combing all the above transforms. i.e.,   
`T0_G = T0_1*T1_2*T2_3*T3_4*T4_5*T5_6*T6_G`   
This gives `T0_G` in the form of variables `q1....6`,   
Now, this total transform is *gripper_link* transform as specified by our DH convention. Shown in *yellow* in the image. However, the 


And, finally, the same total transform can also be computed given the *gripper_link* position and orientation w.r.t. *base_link*.Given,   
```python
pg_0  = [px, py, pz] # End effector position.
p_quat = [qx, qy, qz, qw] # End effector orientation as a quaternion.

# R0_g = end-effector(gripper) rotation transformation(4X4)
R0_g = tf.transformations.quaternion_matrix(p_quat)
D0_g = tf.transformations.translation_matrix(p_quat)

T_total = R0_g*D0_g
```

Now, the total transform computed previously, `T0_G` is *gripper_link* transform specified in DH convention(*yellow*). However, the transform calculated from *gripper_link* position and orientation is in URDF frame(*green*). These are shown in the following image,   
![total-transform-corretion][total-transform-corretion]

So, we need to correct `T0_G` transform  by rotating by `pi` along `Z`, and then by `-pi/2` along `Y`. Thus,    
`T_total = T0_G*rot_z(pi)*rot_y(-pi/2)`, where, `rot_y` and `rot_z` are defined as,    
```python
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
```


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's another image! 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


