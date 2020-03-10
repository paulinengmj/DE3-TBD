## Project Development

HELLO ! This section of the documentation consists of the development from animation creation to image capturing

#### Method Overview
**********
This project was implemented and segmented into 6 main steps

![](methods.png)

###### 1. Animation Sequencing
The model sequence of the butterfly in flight was generated using Rhinoceros3D and the Grasshopper plugin, iteratvely altering a low-poly mesh to give the effect of a butterfly flapping its wings. 
###### 2. Coordinate Generation
The frame sequence was then treated as individual frames, with the coordinates of each one being extracted and processed to eliminate duplicae points. The coordinates were organised to prevent duplicate paths.
###### 3. Motion Planning
A motion plan was then generated for each frame using the coordinate list created in the previous step. Code to iterate over the list of frames and plan each motion was developed. This was simulated using RViz and Gazebo.
###### 4. Simulation
Once simulated successfully, the code was implemented using Franka Emika’s Panda robotic arm. 
###### 5. Implementation
An end-effector attachment containing a controllable LED was simultaneously developed and manufactured to facilitate light painting
###### 6. Image Capture
Once the code successfully ran repeatedly on the Panda robot, long-exposure photgraphs were captured of the motion. These photographs were then collated and displayed in sequence to create an animation of a butterfly in flight.

#### Animation Creation
**********

The butterfly model was simplified to a low-poly model since it was expected that a large number of frames would be needed to form a smooth animation. I was also intended that the focus was on the animation itself and not on the model. As the project was limited in terms of time with the robot, a simplified outline presented the most opportunity to iterate and increase robotic complexity. I.e. A low fidelity model reduces the amount of time necessary for the robot to complete each frame. 

Using slow-motion footage of butterflies in flight, one model for each frame of the animation was created in [Rhinoceros 3D](https://www.rhino3d.com/).  The animation was simplified to 11 frames. The angles of the butterfly wings about the x-axis and the z-axis are approximately sinusoidal. A subtle body wiggle was also added, which is a small, random rotation of the two body triangles about the y-axis. (Angles in reference to Figure B axes).

**_NOTE:_** Grasshopper is a plugin included in Rhino 6, which has built-in functions called “components” to allow for easy data manipulation. 

These are the steps which can be repeated to output the same result for each frame in Grasshopper:

1. Each of the 4 triangles which form a butterfly were deconstructed into their vertices, faces, colours and normals using the “deconstruct mesh” components. This outputs as a list of lists; a list of 4 triangles and each triangle having 3 vertices, where each vertex is an xyz coordinate. 
2. The repeated centre vertices were removed by putting the lists through the “create set” component. 
3. Each sublist was extracted using a “list item” components.
4. Reorder the list in such a way that the robot can move from coordinate to coordinate without retracing the same path twice. 
**_NOTE:_** This is important in order to avoid any brighter spots in the animation frames which would be seen as an anomaly.

5. The result is a list of coordinates in curly brackets.

Simple data manipulation was then carried out in a text editor to put this in the required python form which uses square brackets (replace function in Wordpad, replacing “{“ and “}” for “[“ and “]” respectively.

This method allows for the flexibility of changing the butterfly animation frame by frame without the need to manually change each coordinate. This would also allow for more complexity to be added easily on the butterfly models (ie. higher poly models), at the cost of a longer per-frame drawing time. The Grasshopper definition shown in Figure C is robust enough to deal with having curved lines, and would output these as a higher number of interpolated points to form the curves. Given more time with the robot, this change could be implemented. 

## Motion Planning
**********

Harvey

## Simulation
**********

Luke (Rviz)

## Implementation on Panda
**********

###### Setting Up Franka Emika
First, release the brakes by making sure the external activation device (EAD) is pressed down. Then, in the controller web interface, click to open brakes. A clicking sound will be heard indicating the EAD button needs to be released. After releasing the EAD button, the LED light will turn white, ready to be used. 

###### Controlling Franka & ROS

With libfranka and franka_ros installed in the real-time Linux kernel, the following steps were carried out before launching the pre-programmed code. 

1. Open terminal and type ``` cd franka_ws ```
2. Then, type ``` source /devel/setup.bash ```
3. Split the terminal into 5 terminals.

In the following order, type in these commands in the respective terminals. 



	• Terminal 1: roscore
	• Terminal 2: roslaunch franka_control franka_control.launch 
                  robot_ip:=192.168.0.88 load_gripper:=true
	• Terminal 3: roslaunch panda_moveit_config panda_moveit.
                  launchcontroller:=position
	• Terminal 4: roslaunch panda_moveit_config moveit_rviz.launch 
	• Terminal 5: python ‘NAMEOFCODE’.py

**_NOTE:_** to change the ‘NAMEOFCODE’ as the name of python file to be executed.





#### grass

## Image Capture
**********