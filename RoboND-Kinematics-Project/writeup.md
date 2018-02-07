## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code.


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png
[image4]: ./misc_images/misc4.jpg
[image5]: ./misc_images/misc5.jpg
[image6]: ./misc_images/misc6.png
[image7]: ./misc_images/misc7.png
[image8]: ./misc_images/misc8.png
[image9]: ./misc_images/misc9.png
[image10]: ./misc_images/misc10.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

![alt text][image7]
![alt text][image8]
![alt text][image10]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | - | -
1->2 | - pi/2 | 0.35 | 0.75 | theta1
2->3 | 0 | 1.25 | 0 | theta2
3->4 |  -pi/2 | -0.054 | 0 | theta3
4->5 | pi/2 | 0 | 1.5 | theta4
5->6 | -pi/2 | 0 | 0 | theta5
6->EE | 0 | 0 | 0.303 | theta6


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles.

![alt text][image5]

![alt text][image4]

##### Transformation Matrix

This is the matrix used for deriving transformation matrices for all joints. They can be calculated by just putting in the required alpha, q, d & a values.

```
TF = Matrix([
        [cos(q), -sin(q), 0, a],
        [sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d],
        [sin(q) * sin(alpha), cos(q) * sin(alpha), cos(alpha), cos(alpha) * d],
        [0, 0, 0, 1]
    ])
```

```
T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(DH_Table)

Joint 1: [[ cos(q1), -sin(q1),  0,     0],
          [ sin(q1),  cos(q1),  0,     0],
          [       0,        0,  1,  0.75],
          [       0,        0,  0,     1]]

T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(DH_Table)

Joint 2: [[ sin(q2),  cos(q2),  0,  0.35],
          [       0,        0,  1,     0],
          [ cos(q2), -sin(q2),  0,     0],
          [       0,        0,  0,     1]]

T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(DH_Table)

Joint 3: [[ cos(q3), -sin(q3),  0,  1.25],
          [ sin(q3),  cos(q3),  0,     0],
          [       0,        0,  1,     0],
          [       0,        0,  0,     1]]

T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(DH_Table)

Joint 4: [[ cos(q4), -sin(q4),  0, -0.054],
          [       0,        0,  1,    1.5],
          [-sin(q4), -cos(q4),  0,      0],
          [       0,        0,  0,      1]]


T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(DH_Table)

Joint 5: [[ cos(q5), -sin(q5),  0,      0],
          [       0,        0, -1,      0],
          [ sin(q5),  cos(q5),  0,      0],
          [       0,        0,  0,      1]]

T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(DH_Table)

Joint 6: [[ cos(q6), -sin(q6),  0,      0],
          [       0,        0,  1,      0],
          [-sin(q6), -cos(q6),  0,      0],
          [       0,        0,  0,      1]]

T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(DH_Table)

(Joint 7)
Gripper: [[       1,        0,  0,      0],
          [       0,        1,  0,      0],
          [       0,        0,  1,  0.303],
          [       0,        0,  0,      1]]
```

##### IK Position and Orientation

**IK Position** : The first three joints are wrist-center and are used to change the position of the robot. Calculations are done by using Pythagoras theorem on each edge and using the equations to get the angle value.

```
theta1 = atan2(WC[1], WC[0])
theta2 = pi / 2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
theta3 = pi / 2 - (angle_b + 0.036)
```

**IK Orientation** : The other three joints are used in order to change orientation of the end effector. While Position change is required to reach the closest area towards the object to be picked, Orientation change is needed to align the gripper in correct direction so that the object can be picked up.

```
theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])
theta5 = atan2(sqrt(R3_6[0, 2] * R3_6[0, 2] + R3_6[2, 2] * R3_6[2, 2]), R3_6[1, 2])
theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])
```


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]
