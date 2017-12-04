## Robot Arm Kinematics Project 
-----------------------------------------
[//]: # (Image and equiation References)
[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png
[image4]: ./misc_images/gazebo_robo_arm.jpg
[image5]: ./misc_images/arbiz_robo_arm.png
[image6]: ./misc_images/dh_parameters.png
[equ1]: ./misc_images/transform-single.png
[equ2]: ./misc_images/transform-comb.png
[equ3]: ./misc_images/angle_d.png
[equ4]: ./misc_images/angle_e.png
[equ5]: ./misc_images/R-calc.png
[equ6]: ./misc_images/R_3_6.png
[equ7]: ./misc_images/R_3_6_simpl.png
[equ8]: ./misc_images/R_rpy.png
[schematic1]: ./misc_images/Theta1.png
[schematic2]: ./misc_images/Theta2-3.png
[schematic3]: ./misc_images/Theta3.png


##### *Robotic Arm / Kinematics Pick & Place*

![gazevo][image4]

![rviz environment][image5]
###### Gazebo/rviz simulation of a robot arm,  __kuka kr210__.

###### This project is part of a program for Udacity students, on the Robotics Nanodegree Program. More information on How to set up the project in the last section of this document.


### Forward Kinematics
 
Before We start coding any instruction to move our robot arm. It is necessary to calculate the rotation and translation necessary to do it.

First, we need to know our robot better, separating the robot arm by joins and links, considering the distance between each join. We also, need to divide the arm into frames that allow us to make the calculation easier. normally you can find this information in the manufacturer's manual. but, In this case, we are going to be looking in our [kr210.urfd.xacro](https://github.com/csilver2/RoboND-Kinematics-Project/tree/master/kuka_arm/urdf) File to get this information.

The URFL file contains useful references about the coordinates of our robot arm,  in the code below you can see an example of the join 1 with 3D coordinates referenced to the base join or join 0:

```python
<joint name="joint_1" type="revolute">
    <origin xyz="0 0 0.33" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="link_1"/>
    <axis xyz="0 0 1"/> 
```

Once all the 3D coordinates for each link are found, we can calculate the Forward Kinematics, which is a procedure that uses mathematics to locate our end effector. But, First, we need to change date taken from the URL accordingly to the Denavit-Hartenberg method.

We can define the steps to complete forward Kinematics in the following order.
1. __Define the DH diagram and parameter table__
2. __Create individual transform matrices__
3. __Extract rotation matrix for EE__

#### 1.  Denavit-Hartenberg (DH) Parameters

Once all the 3D coordinates for each link are found, we can calculate the Forward Kinematics, which is a procedure that uses mathematics to locate our end effector.

![DH parameters diagram][image6]

DH parameters

|ID   | αi-1 		   | ai-1   | di     | θi 		 |
|:---:|:------------------:|:------:|:------:|:-----------------:| 
|    1|                  0 |      0 |   0.75 |      	      θ1 |
|    2|     	      -π/2 |   0.35 |      0 | 		 θ2 -π/2 |
|    3|                  0 |   1.25 |      0 |      	      θ3 |
|    4|       	      -π/2 | -0.054 |   1.50 |     	      θ4 |
|    5|       	       π/2 |      0 |      0 |     	      θ5 |
|    6|      	      -π/2 |      0 |      0 |      	      θ6 |
|   EE|                  0 |      0 |  0.303 |                 0 |

###### Code implementation:

```python
dht = {alpha0: 	 0,   a0:  	 0,    d1:  d02,	q1: 	  q1,
       alpha1: ann,   a1:  a02,        d2:    0,        q2: ann + q2,
       alpha2: 	 0,   a2:  a23,        d3:    0,	q3:	  q3,
       alpha3: ann,   a3:  a34,        d4:  d35,	q4:       q4,
       alpha4: anp,   a4:  	 0,    d5:    0,	q5: 	  q5,
       alpha5: ann,   a5:  	 0,    d6:    0,	q6: 	  q6,
       alpha6: 	 0,   a6:  	 0,    d7:   dg,        q7:  	   0}
```

#### 2. Homogeneus transform matrix 

With the DH parameters, we can calculate the individual homogeneous transform matrices for each joint.

you can see the matrix used for each homogeneous transform on the image below.
![convine transform matrices][equ1]

###### Code implementation:

```python
def Ind_transform(q, alpha, d, a): #individual transform matrix

			T0_N = Matrix([[        cos(q),           -sin(q),           0,     a],
            	   	   [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
               		   [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
               		   [                 0,                 0,           0,            1]])

			return T0_N

		# Individual transform for each link
		T0_1 = Ind_transform(q1, alpha0, d1, a0).subs(dht)
		T1_2 = Ind_transform(q2, alpha1, d2, a1).subs(dht)
		T2_3 = Ind_transform(q3, alpha2, d3, a2).subs(dht)
		T3_4 = Ind_transform(q4, alpha3, d4, a3).subs(dht)
		T4_5 = Ind_transform(q5, alpha4, d5, a4).subs(dht)
		T5_6 = Ind_transform(q6, alpha5, d6, a5).subs(dht)
		T6_G = Ind_transform(q7, alpha6, d7, a6).subs(dht)
```

Once we have each homogeneous transform we can calculate the total homogeneous transform for our kuka arm by multiplying each transform together.
![homogeneus transform multiplication][equ2]

#### 3. Extract rotation matrix for EE.

There is an orientation different from the gripper on the URFD file and the DH convention frames. to compensate this we can add a rotation to the final homogeneous transform : 180 Z axis, -90 Y Axis.

###### Code implementation.
```python
R_z = Matrix([[             cos(pi),            -sin(pi),            0,              0],
           [                        sin(pi),            cos(pi),             0,              0],
           [                        0,                  0,                   1,              0],
           [                        0,                  0,                   0,              1]])

		# then rotate around y-axis by -pi/2 radiants or -90 degrees
		R_y = Matrix([[             cos(-pi/2),         0,                   sin(-pi/2),     0],
           [                        0,                  1,                   0,              0],
           [                        -sin(-pi/2),        0,                   cos(-pi/2),     0],
           [                        0,                  0,                   0,              1]])

		#Calculate total correction factor of the urdf file on the end effector
		R_corr = simplify(R_z * R_y)
```

### Inverse Kinematics

Once We know the end effector position, our objective is to calculate the angles for each join. This procedure is called inverse kinematics. Using the diagram [schematic1] we can have a better idea on how to find the angles by using the cosine law.

![robo arm joins and angles schematic][schematic1]


__Steps to complete Inverse Kinematics__
1.__Calculate theta 1, 2, and 3.__
2.__Calculate rotation matrix from link 0 to link 3__
3.__Calculate thetha 4, 5, and 6__.

#### 1.  Calculating Theta 1, 2, and 3
The reference frame O4, O5, and O6 intersect at the same coordinate. So we choose a frame as Wrist Center (WC), which is a method that simplifies the mathematic calculation in inverse kinematics named **closed-form** , which then allow us to solve the angles θ1 to θ3 using cosine law. 

#### 1.1. Theta 1(**θ1**)
Here is a diagram that shows the top view of the kuka arm with points that indicate each frame.

We can get **θ1** by calculathing by rotating O1 about its Z-axis. 
![robo arm joins and angles schematic][schematic1]

#### 1.2. Theta 2-3(**θ2**, **θ3**)
the diagram below shows the side view of the kuka arm, and the mothod to calculate θ2 to θ3 with cosine law:
![robo arm joins and angles schematic][schematic2]

θ2 is the angular distance between O2 and its position in the Z2 axis, thus it can be calculated finding the angle a and angle d once we have these angles we substract these values from the  90 grades angle, and the result is the theta angle.
```python
#			    ---------------- angle_d ------------------------------------
theta2 = pi / 2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
```
The angle d can be calculated taking WC and O2 triangle then apliying the next equation:
![angle d equation][equ3]

Similar to θ2 we can calculate θ3 by subtract from the 90 degrees angle, the angle b and angle e.
![robo arm joins and angles schematic][schematic3]

```python
#caclulate theta 3
#			    angle_e
theta3 = pi / 2 - (angle_b + 0.036)
```

angle e can be calculated with the followed equation:
![angle e quation][equ4]

#### 2. Calculating rotation matrix and theta 4 to 6
Finally to solve the inverse orientation problem we need to find **θ4**, **θ5**, and **θ6**, by using the resultan rotation:
![equ5][equ5]

We then analyze the left and right hand sides of the equation above independently and solve for **θ4**, **θ5**, and **θ6**.

We know that R0_6 is the overall (roll, pitch, yaw) ratation between the base link and the end effector link so we uset to calculate R_3_6

Comparing LHS (Left Hand Side of the equation) with RHS (Right Hand Side of the equation) will result in equations for joint 4, 5, and 6.


################################################################

## Setting Up the Environment

[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)
# Robotic arm - Pick & Place project

Make sure you are using robo-nd VM or have Ubuntu+ROS installed locally.

### One time Gazebo setup step:
Check the version of gazebo installed on your system using a terminal:
```sh
$ gazebo --version
```
To run projects from this repository you need version 7.7.0+
If your gazebo version is not 7.7.0+, perform the update as follows:
```sh
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install gazebo7
```

Once again check if the correct version was installed:
```sh
$ gazebo --version
```
### For the rest of this setup, catkin_ws is the name of active ROS Workspace, if your workspace name is different, change the commands accordingly

If you do not have an active ROS workspace, you can create one by:
```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

Now that you have a workspace, clone or download this repo into the **src** directory of your workspace:
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/udacity/RoboND-Kinematics-Project.git
```

Now from a terminal window:

```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ sudo chmod +x target_spawn.py
$ sudo chmod +x IK_server.py
$ sudo chmod +x safe_spawner.sh
```
Build the project:
```sh
$ cd ~/catkin_ws
$ catkin_make
```

Add following to your .bashrc file
```
export GAZEBO_MODEL_PATH=~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/models

source ~/catkin_ws/devel/setup.bash
```

For demo mode make sure the **demo** flag is set to _"true"_ in `inverse_kinematics.launch` file under /RoboND-Kinematics-Project/kuka_arm/launch

In addition, you can also control the spawn location of the target object in the shelf. To do this, modify the **spawn_location** argument in `target_description.launch` file under /RoboND-Kinematics-Project/kuka_arm/launch. 0-9 are valid values for spawn_location with 0 being random mode.

You can launch the project by
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ ./safe_spawner.sh
```

If you are running in demo mode, this is all you need. To run your own Inverse Kinematics code change the **demo** flag described above to _"false"_ and run your code (once the project has successfully loaded) by:
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ rosrun kuka_arm IK_server.py
```
Once Gazebo and rviz are up and running, make sure you see following in the gazebo world:

	- Robot
	
	- Shelf
	
	- Blue cylindrical target in one of the shelves
	
	- Dropbox right next to the robot
	

If any of these items are missing, report as an issue.

Once all these items are confirmed, open rviz window, hit Next button.

To view the complete demo keep hitting Next after previous action is completed successfully. 

Since debugging is enabled, you should be able to see diagnostic output on various terminals that have popped up.

The demo ends when the robot arm reaches at the top of the drop location. 

There is no loopback implemented yet, so you need to close all the terminal windows in order to restart.

In case the demo fails, close all three terminal windows and rerun the script.

