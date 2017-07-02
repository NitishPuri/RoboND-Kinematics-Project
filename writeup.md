## Project: Kinematics Pick & Place

[//]: # (Image References)

<!--[image1]: ./misc_images/misc1.png-->
[image2]: ./misc_images/misc2.png
[image3]: ./misc_images/misc3.png
[annotated_image]: ./misc_images/annotated_image.png
[urdf_annotation]: ./misc_images/urdf_annotation.png
[dh-transform-matrix]: ./misc_images/dh-transform-matrix.png

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

![Modified DH Transformation matrix][dh-transform-matrix]


Here's | A | Snappy | Table
--- | --- | --- | ---
1 | `highlight` | **bold** | 7.41
2 | a | b | c
3 | *italic* | text | 403
4 | 2 | 3 | abcd

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's another image! 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


