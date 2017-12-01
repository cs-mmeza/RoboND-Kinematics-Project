## Robot Arm Kinematics Project 
-----------------------------------------
[//]: # (Image and equiation References)
[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png
[image4]: ./misc_images/gazebo_robo_arm.jpg
[image5]: ./misc_images/arbiz_robo_arm.png
[image6]: ./misc_image/dh_parameters.jpg
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

![image3][image3]

![rviz environment][image5]
###### Gazebo/rviz simulation of a robot arm,  __kuka kr210__.

###### This project is part of a program for Udacity students, on the Robotics Nanodegree Program. More information on How to set up the project in the last section of this document.


### Forward Kinematics
 
Before We start coding any instruction to move our robot arm. Is necessary to calculate the rotation and translation to move it. This information can be found in the manufacturer's manual of the robot arm, but for this simulation, the coordinates needed are found in [kr210.urfd.xacro](https://github.com/csilver2/RoboND-Kinematics-Project/tree/master/kuka_arm/urdf)

Once all the 3D coordinates for each link are found, we can calculate the Forward Kinematics, which is a procedure that uses mathematics to locate our end effector.

We can define the steps to complete forward Kinematics in the following order.
1. __Define the DH parameter table__
2. __Create individual transform matrices__
3. __Extract rotation matrix for EE__

#### 1.  Denavit-Hartenberg (DH) Parameters

With this method, we only require four methods to calculate forward kinematics and inverse kinematics.

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

#### 2. Homogeneus transform matrix combination

![convine transform matrices][equ1]

#### 3. Extract rotation matrix for EE.

Using [equation 1] we can calculate the rotation and translation between each join. To calculate a total homogenous transform for the robot and know the location of the robot we calculate the transform matrix for each join and then we calculate the dot product for all the transforms. For a simplified view check the equation below.

![homogeneus transform multiplication][equ2]


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

