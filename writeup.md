## Project: Kinematics Pick & Place

[//]: # (Image References)

<!--[image1]: ./misc_images/misc1.png-->
[annotated_image]: ./misc_images/annotated_image.png
[urdf_annotation]: ./misc_images/urdf_annotation.png
[dh-transform-matrix]: ./misc_images/dh-transform-matrix.png
[dh-transform]: ./misc_images/dh-transform.png
[dh-convention]: ./misc_images/dh-convention.png
[total-transform-corretion]: ./misc_images/total-transform-corretion.png
[theta1]: ./misc_images/theta1.png
[theta23]: ./misc_images/theta23.png
[errors_1]: ./misc_images/errors1.png
[errors_2]: ./misc_images/errors2.png
[errors_3]: ./misc_images/errors3.png
[errors_4]: ./misc_images/errors4.png
[the-spiral]: ./misc_images/the-spiral.jpg

### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

We can use the following images for reference.   
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
**Fig. 3 :** Modified DH convention axes assignment and parameters.

From the above image it is clear thet the total transform between `link(i-1)` and `link(i)` can be thought of as a *rotation* by `alpha(i-1)` along `X(i-1)`, *translation* by `a(i-1)` along `X(i-1)`, *rotation* by `q(i)` along `Z(i)`, and finally *translation* by `d(i)` along `Z(i)`. That is,   

![DH Transform][dh-transform]   
**Fig. 4 :** Transform between two links in Modified DH convention.

Which, when expanded analytically turns out to be,   

![Modified DH Transformation matrix][dh-transform-matrix]   
**Fig. 5 :** Transform between two links in Modified DH convention in expanded form.   
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
D0_g = tf.transformations.translation_matrix(pg_0)

T_total = R0_g*D0_g
```

Now, the total transform computed previously, `T0_G` is *gripper_link* transform specified in DH convention(*yellow*). However, the transform calculated from *gripper_link* position and orientation is in URDF frame(*green*). These are shown in the following image,   
![total-transform-corretion][total-transform-corretion]
**Fig. 6 :** Showing the gripper link frame in *URDF* and *DH* convention.

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


Now th IK part. 
The  last three joints `q4, q5, q6` don't affect the position Wrist Center(`O5`), hereby refered to as `WC` position.(this can be confirmed by runnning the Forward kinematics demo.) This is very convinient for us in decoupling the IK problem into a position and orienation problem where we can first compute the position of the `WC`(which gives us the first three joint angles `q1, q2, q3`) and orienatation of the wrist , which gives us the last three joints `q4, q5, q6`. Detailed explaination follows,   

* Given End effector position and orienation, we calculate the the wrist center as follows,
```python
# rwc_0 = wrist-center position w.r.t. base_link(O_0)
rwc_0 = pg_0 - (d_g*(R0_g*z_g))
```
Where,   
`pg_0` = End effector position received.   
`p_quat` = End effector orienataiton received as quaternion.   
`d_g = s[d7]` = Displacement of end-effector from wrist center(along z).   
`R0_g = tf.transformations.quaternion_matrix(p_quat)` = End effector rotation matrix.   
`z_g = rot_z(pi)*rot_y(-pi/2)*([0, 0, 1])` = gripper frame z axis in DH convention.   

So, the above equation displaces `pg_0` by `-d_g` in the z direction.

* Calculate `q1` given `WC`.   
![Theta 1][theta1]
**Fig. 7 :** Oblique view of the arm, when `q1 != 0`, all other angles are assumed zero.

From the above image, `q1` can be calculated by projecting `WC` on `X0-Y0` plane and calculating angular displacement from `X0`. i.e.   
```
theta1 = atan2(rwc_0[1], rwc_0[0])
```
* Calculate `O2` position according to calculated `q1`.
```python
# position of O2 w.r.t. base_link in zero configuration, i.e. when q1 = 0
pO2_0 = Matrix([[s[a1]], [0], [s[d1]]])
# Rotate pO2_0 w.r.t. Z by theta1
pO2_0 = rot_z(theta1)* pO2_0
```

*  Consider `triangle(O2, O3, WC)`.   
*Note :* Points `O0`, `O1`, `O2`, `O3` and `O5/WC` now lie in the same plane, defined by rotatingplane `X0Z0` about `Z0`.
![theta23][theta23]
**Fig. 8 :** Schematic of arm in the plane containing triangle `O2`, `O3`, `WC`. Various angles required dimensions are also shown.   

In the figure, position `O2`, `WC` are known and length `A`, `B` and `C` are known. 
```python
# O2 , WC are known
# O3 = unknown
# Distance between O2 and O3 = a2(in figure 1)
A = s[a2]

# Distance between O2 and O5/WC
pO2towc_0 = rwc_0 - pO2_0
B = pO2towc_0.norm()

# Distance between O3 and O5/WC = (d4^2 + a3^2) in figure 1
C = np.sqrt(s[d4]*s[d4] + s[a3]*s[a3]) 

# Offset angle between the Y3 axis line(O3, O5), -q31 in figure
beta_prime = atan2(s[a3], s[d4])   # From Fig. 1
```
Now, we can apply *Law of cosines* in `triangle(O2, O3, WC)`.   
```
In any triangle(A, B, C)
c.c = a.a + b.b - 2.a.b.cos(alpha),
where alpha = angle(BAC) and `.` represents multiplication
```

So, 
```python
# applying cosine rule `C^2 = A^2 + B^2 -2ABcos(gamma)`
# angle(O3, O2, O5), q21 in figure.
gamma = np.arccos(float((A*A + B*B - C*C)/(2*A*B)))

# angle(O2, O3, O5), q32 in figure 
beta = np.arccos(float((A*A + C*C - B*B)/(2*A*C)))
```
* Find `theta2`   
Next, we need to compute `theta2`, which can be thought of as the angle between `link 2` direction i.e. `dir(O2, O3)` and `X2`. Also, `dir(O2, O3)` can be calculated by rotating `dir(O2, WC)` by `-gamma` along `Z2`.   
We can get `X2` and `Z2` by substituting `q1=theta1` (calculated above) in `T0_2` and multiplying the result by X(1, 0, 0) and Z(0, 0, 1) respectively. *Note :* `T0_2 = T0_1*T1_2`.   
So,   
```python
# z_2prime is the Z2 axis when q1 = theta1, this does not depend upon q2 
z_2prime = T0_2.subs({q1:theta1}).dot([[0], [0], [1], [0]])

# Rotate pO2towc_0 by gamma along z_2prime
z2_rotation =  tf.transformations.rotation_matrix(-gamma, z_2prime)
# quaternion_about_axis(gamma, z_2prime[0:3])
# tf.transformations.quaternion_from_matrix(z2_rotation)
a_dir = z2_rotation * pO2towc_0
a_dir = a_dir.normalized()

# Compute theta2
X2_prime = self.T0_2.subs({q1:theta1, q2:0}).dot([[1], [0], [0], [0]])
theta2 = np.arccos(float(np.dot(X2_prime, a_dir[0:4]) ))
```
* Find `theta3`   
`theta3` is simply the deviation of `angle(O2, O3, WC)` from `pi/2 - q31`.i.e.   
`theta3 = (pi/2 - q31) - q32`, But `q31 = -beta_prime` and `q32 = beta`, So,   
```python
theta3 = (pi/2 + beta_prime) - beta
```

* Now, we need to calculate `q4, q5, q6`. For this we calculate the rotation matrix `R3_6` from our total transform and calculated angles *numerically*(from end effector position/rotation) and *symbolically*(from DH parameters). We then compare the two representations to calculate plausible values of the last three joint angles. So,   

**Symbolically :** Just combine the symbollic transformations for individual links from link 3 to 6.

```python
# Extract the rotation component of the matrix,cos that's what we want
>>> R3_6_sym = (T3_4*T4_5*T5_6)[:3,:3] 
[[-s4s6 +c4c5c6,-s4c6 -s6c4c5,-s5c4],
 [         s5c6,        -s5s6,   c5],
 [-s4c5c6 -s6c4, s4s6c5 -c4c6, s4s5]]
# Where, s = sin, c =cos, 4,5,6 = q4,q5,q6
# So that, -s5c6 = -sin(q5)cos(q6)
```

**Numerically :** Since,    
   
where ,   
`` . So,   
```python
# Calculate R3_6
R0_g(corrected) = R0_6*R_corr
# Where
R_corr = rot_z(pi)*rot_y(-pi/2)
# So
R0_g = R0_3*R3_6*R_corr
R0_3.T*R0_g*R_corr.T = R3_6

# And,
R0_3 = T0_3[0:3,0:3]   # Extract the rotation matrix
R3_6 = R0_3.transpose()* Matrix(R0_g)*self.R_corr.transpose()
```   
Finally we can evaluate this Matrix numerically by substituting `q1, q2 and q3` calculated above
```
R3_6 = R3_6.subs({q1: theta1, q2:theta2, q3: theta3})
```

Now, we need to select suitable terms from the matrix to compute `q4, q5, q6`.
We just follow the strategy of selecting the simplest terms to give to `atan2` function for a given angle. i.e.   
`theta4 = atan2(R3_6[2,2], -R3_6[0, 2])`, Since,   
`R3_6[2,2]/-R3_6[0, 2] = sin(q4)sin(q5)/sin(q5)cos(q4) = tan(q4)`   

`theta5 = atan2(sqrt(R3_6[0, 2]*R3_6[0, 2] + R3_6[2, 2]*R3_6[2, 2]), R3_6[1, 2])`, Since,   
`sqrt(R3_6[0, 2]*R3_6[0, 2] + R3_6[2, 2]*R3_6[2, 2]) = sin(q5)`,   
`sqrt(R3_6[0, 2]*R3_6[0, 2] + R3_6[2, 2]*R3_6[2, 2])/R3_6[1,2] = sin(q5)/cos(q5) = tan(q5)`   

And finally, 
`theta6 = atan2(-R3_6[1,1], R3_6[1, 0])`, Since,   
`-R3_6[1,1]/-R3_6[1, 0] = sin(q5)sin(q6)/sin(q5)cos(q6) = tan(q6)`   

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

Most of the code has already been discussed in the previous sections, here i will give an overview of the changes i made to the template `IK_server.py` provided to make things work efficiently.

* My implementation is based on first computing symbolic transforms for each individual link using the parameters provided. This computation can be time consuming to be run every time an IK analysis needs to be  performed. So, first and foremost I crated a class `KukaIKSolver` which does these symbollic simplifications at initialization once.   
Here is the high level class description:
```python
class DummyReq(object):
    ### Dummy request that works as a proxy for request object received from the simulator 
    ### This really helped in quick debugging to test the code on known values and validation.
    ### Testing approach discussed in detail later.

class KukaIKSolver(object):
    def __init__(self):
        # Create T0_1,.... T6_G
        # simplify and compute T0_G
        # Create correction transformation for gripper link from DH to URDF
        # Compute symbollic transform for R3_6
        # temporarily memorize  theta4 and theta6, explained later.
        pass

    def performFK(self, theta_t):
        # @param theta_t : list of theta values corresponding to joint angles 1, 2, ..6
        # Perform Forward kinematics analysis on with the given joint angles by 
        # substituting theta values in the already computed symbollic transforms to

        # Return the computed end effector position and orientation wrapped in a single object.
        return DummyReq(EE_pos, EE_orientation)

    def generateRandomReq(self):
        # generate a random set of angles in the range provided by URDF.
        # Perform FK on the generated angles and return the generated dummy request to perform IK on.
        pass

    def handle_calculate_IK2(self, req):
        # This is based on the handle_calculate_IK provided in the template code.
        # For every pose in the request
            # Compute WC
            # Compute theta1
            # Solve triangle(O2, O3, WC)
            # Compute theta2, theta3
            # Compute R3_6
            # Compute theta4, theta5 and theta6 using already defined equations.
            # Check if all the angles are within the range specified in URDF. If not show an error.
            # Reduce effective range of theta4 and theta6.
            # Fix theta4 and theta6 based on last computed theta values for these angles.
            # Perform FK on the computed angles to get the effective EE position
        # Dump the error in a file to be used later for displaying error graphs.
        # Send back the IK response.
```
Also, there are some helper functions, listing the most important ones   
```python
def rot_y(q):
    # return rotation about Y by q
def rot_z(q):
    # return rotation about Z by q

def createMatrix(alpha, a, q, d):
    # return DH transform between two consecutive links

def createPlot(errors):
    # plot a bar graph of the provided errors and save to a file named "plots/errors {current_time}.png"

def saveErrors(errors):
    # save the errors to disk by dumping using `np.savetext()` to a file named
    # "plots/errors {current_time}"
```

##### Calculatin errors
* Errors are computed by calculating the distance between the received EE position and position computed after performing FK on the angles computed by IK. 
```python
error = (pg_0 - pos_FK).norm()
```

Showing the result of some of the plots created by error dumps.
![er1][errors_1]  
![er2][errors_2]
![er3][errors_3]
![er4][errors_4]

* The above plots can be created by calling a separate script `plotTest.py` available in the same directory 
```sh
$ python plotTest.py plots/{filename}
```

* I had to create a separate script since plotting errors was somehow causing the arm to behave differently than when the errors were not plotted, and also the `IK_server.py` just crashed after completing the seconf request consistently. So,I had to dump the computed errors. Possibly there is some bug in my plotting code, or i need to clean up the `plt` state. Need to figure this one out.

##### Restricting `theta4` and `theta6` values.
* After the complete implementation  I saw that most of the time the wrist was just rotating about either joint 4 or joint 6.  
How I understand it is that in the simulation that moves the arm between the poses, it tries to reach all the joint angles as close as possible constrained by some specified joint angular velocity limits. And because of large variations in 4 and 6 joint angles it spends most of its time adjusting these angles and hence is not able to correctly follow the calculated trajectory properly. 
To fix this, first i reduced the effective range of the angles from (-350, 350) to (-180,180). i.e. wrapped the angle into a smaller range.
```python
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

theta4 = reduceEffectiveRange(theta4, "theta4")
theta6 = reduceEffectiveRange(theta6, "theta6")
```
![spiral][the-spiral]   
**Fig. 9:** *The spiral shows the complete range of both `theta4` and `theta6` (-350, 350). Also, there are two possible solutions for most of the orientations of the joints. The **bold** spiral shows the effective range after reduction(-180, 180). Once this is done, the rediced angle is compared to the old angle, and corrected so that it doesnt go for a more than 180 rotation.* 

This, did not have any effect on the unnecessary wrist rotation. However, as can be seen in the figure, now i have an opportunity to change the computed angle so that the difference between `new` and `old` is not more than `pi`. 
```python
def angleCorrection(theta, old_theta):
    d = theta - old_theta
    if d > np.pi:
        return theta - 2*np.pi
    elif d < -np.pi:
        return theta + 2*np.pi
    else:
        return theta


theta4 = angleCorrection(theta4, old_theta4)
theta6 = angleCorrection(theta6, old_theta6)

old_theta4 = theta4
old_theta6 = theta6
```



##### Possible improvements
* I used tf.transformations library wherever possible.It doesn't work with sympy symbols. It expects numerical values(numpy arrays). However, operations between numpy arrays and sympy Matrices are not implicit. I had to explicitly convert in between the two at various places and that caused a lot of trial and error to see which `type` of matrix should i use. Also, i mixed up `3X3` and `4X4` homogeneous matrices and had explicitly convert between the two to make sure the operations are compatible. These things can definitely be cleaned up, that would also make the code look a lot clean than it looks right now.

* I only show an console error when angles go out of range, but since i have not seen any instance of out of range error(neither in simulation, nor in randomized testing), i did not add any conditions to fix it. I would definitely have done that if i saw fairly regular out of range errors. But it should be handled nonetheless.

* I found some poses (during randomized testing), in which when `O2` and `WC` lie in opposite quadtrants, `theta1`(actual, according to `O2`), and `theta1`(computed, according to `WC`) are opposite to each other. This never happened during simulation with ROS, however, i was able to create a set of `thetas` during randomized testing that led to this situation. I guess this was because my test set was larger than the set of angles possible to get from IK. I was not able to follow this problem anymore due to time limitations, and also this only happened only rarely during randomized tests. 



