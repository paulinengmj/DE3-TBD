## Code Development

This section talks about how the final python code works and why it is set up in this way. The final code is available on GitHub [here](https://github.com/HarveyU1/LightPaintingRobot). 

&nbsp;

#### Code Structure
**********
There are two main python files which are used to get the panda robot to light paint.

-```pandaLightPaint``` is the main python file which is executed from ROS to carry out motion planning for each frame of the animation and then executes this motion plan by moving the end effector and controlling the gripper position.

-```frames``` is the python file in which the cartesian based trajectory is stored for a complete animation. This is imported by pandaLightPaint and informs the motion plan which is calculated.
##
&nbsp;

&nbsp;
#### Main Code (pandaLightPaint)
_____
The main python code that is launched from the terminal.


###### Set Up
**********
First, the required python modules are imported, ensure that all of these are installed in your python installation within ROS. Note ROS specific modules such as different message formats which allow communication between nodes.

```python
#!/usr/bin/env python
#ROS specific
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String, Float64MultiArray, MultiArrayDimension, Float64
from moveit_commander.conversions import pose_to_list
```

Next, the generic python packages are imported. math.pi is required for robot angle calculations which are based in radians

```python
from math import pi
import tf
```

Finally, the list framelist is imported from the frames python file which is stored alongside pandaLightPaint
```python
from frames import framelist
```

&nbsp;

###### Modifiable variables
**********
These are the variables which can be changed to modify the behaviour of the robot. Some of these are useful to modify when troubleshooting for problems as they can often help prevent errors.


The offset coordinates are the cartesian coordinates in metres of the starting position for all frame drawing. The frame coordinates, in framelist, are provided w.r.t to a central local origin point. In the case of the butterfly, this is the centre of its body. The offset coordinated oOffCords specify the location of the frames local coordinate system relative to the robots coordinate system which has an origin located at the base of the arm. By default, the offset coordinates are set to 0.5 metres in front of the robot(x-axis) and 0.5 metres height above the base of the robot(z-axis). This always for a good amount of redundancy when drawing frames as the arm is well within the dexterous workspace.

```python
oOffCords = [0.5,0,0.5]
```


The offset rotation specifies the desired pitch, roll and yaw of the end effector during light painting. When motion planning this is added as a constraint to ensure the end effector is always facing in the same direction. This means that the light will always be facing towards the camera and prevents distortion of the frame draw. By default, this is set to a pitch of pi radians and yaw of -pi/4 radians which correspond to custom torch attachment facing directly forward. These values can be changed if the camera is positioned differently.

```python
oOffRots = [pi,0,-pi/4]
```



The shake angle simply specified the angle in radians by which the end effector will be offset in yaw during the shake manoeuvre. Generally, this will not need to be changed unless you want the shake to be fast in which case the angle should be reduced.
```python
shakeAngle = pi/8
```

Frame Scale is used to adjust the dimensions of the frames within framelist for the animation. Depending on the unit of scale for the animation the frame coordinates need to be scaled to fit with the dexterous workspace of the robot. If the coordinate is outside the workspace then there will either be a motion planning error or the robot will fail to execute the motion plan. In this, the framescale should be reduced to ensure all coordinates are within the acceptable range. Generally, the framescale can be determined by finding the value of the largest coordinate then choosing a framescale which multiplies this value to equal 0.3. This ensures that the frame will all be drawn within a 0.6-metre sized box with its centre and the position of the offset coordinate.

framescale = 0.3 / Largest Coordinate

This value is not automatically calculated as users may want to experiment with drawing larger frames or drawing smaller frames to save time. Additionally, if multiple frames files are used it may be required that these are all scaled by the same value.

```python
frameScale = 0.004
```



Processing time is simply the length of time in seconds between the drawing of each frame. This allows time for the camera to process the long exposure photo. This value can be changed to suit the processing time for whichever camera is being used.
```python
processingTime = 20
```
Light on and light off distances are the distances in metres which the end effector grabber will open and close to when turning the light on and off during frame drawing. The default values work well for the custom end effector light specified in this documentation. If a different torch design is used these values can be changed as required.
```python
lightOnDist = 0.03
lightOffDist = 0.05
```

&nbsp;

###### Main Execution Set Up
**********
This is the main section of the code which is executed when the program is launched from the terminal. It includes the initial set up as the main loop which is responsible for drawing each frame.

To begin with rospy a new node is created for the panda light controller. Next, the motion planing is initialised with the group name being assigned.

```python
rospy.init_node('panda_light_paint_controller', anonymous=True)

#Initialise motion planning in RVIZ
moveit_commander.roscpp_initialize(sys.argv)	
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "panda_arm"
group = moveit_commander.MoveGroupCommander(group_name)
```

A publisher is created for the controlling the gripper. The message format for the gripper is also specified as a 2d array of float values.
```python
gripper_publisher = rospy.Publisher('/franka/gripper_position_controller/command', Float64MultiArray, queue_size=1)
gripper_msg = Float64MultiArray() #
gripper_msg.layout.dim = [MultiArrayDimension('', 2, 1)]
```
    
Finally the goToPose function is called to set up the robot in it's starting configuration with the end effector at the offset positition at the offset rotation values.

```python
goToPose(oOffRots[0],oOffRots[1],oOffRots[2],oOffCords[0],oOffCords[1],oOffCords[2])
```

&nbsp;

###### Main Execution Loop
**********
The main execution loop will be carried out for each frame of the animation in framelist. Each time this loop is executed a frame will be drawn. The loop starts with a terminal message informing the user to open the shutter on the camera. Next, the lightOn function is called which closes the grippers to turn on the torch held by panda. The goOnPath function is executed which draws one frame of the animation. The light is then turned off and a terminal message is printed informing the user to end the current photo. Finally, a delay is carried out to allow time for the camera to process the image before the next frame.

```python
for frame in framelist:

	#User inform message to start photo
	print("Open Shutter")

	#Turn on the light by closing the end effector
	lightOn()

	#Wait 1 second
	rospy.sleep(1)

	#Execute frame motion
	goOnPath(frame)

	#Turn off the light by opening the end effector
	lightOff()

	#User inform message to end photo
	print("Close Shutter")

	#Wait time so that camera has enough time to process image before next frame
	rospy.sleep(processingTime)
```

&nbsp;


###### goToPose Function
**********
This function takes the input arguments roll, pitch and yaw, as well as x,y and z coordinates, and will calculate and execute a motion plan to get the robot in the correct pose. This is done by first clearing the old pose targets. Then converting the rotation values into a quaternion which is used to set the new values for the origin pose (which is the target pose). The motion plan is then calculated and executed. If a motion plan cant be found for the chosen pose values and an error message will be displayed in the terminal.
```python
def goToPose(roll,pitch,yaw,xCord,yCord,zCord):
	group.clear_pose_targets()

	quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
	origin_pose = geometry_msgs.msg.Pose()
	origin_pose.orientation.x = quaternion[0]
	origin_pose.orientation.y = quaternion[1]
	origin_pose.orientation.z = quaternion[2]
	origin_pose.orientation.w = quaternion[3]
	origin_pose.position.x = xCord
	origin_pose.position.y = yCord
	origin_pose.position.z = zCord

	group.set_pose_target(origin_pose)
	plan = group.go(wait=True)
```

&nbsp;

###### shakeHand Function
**********
This function will execute a number of handshakes specified by the quantity input argument. These handshakes can be used to indicate the start and end of frames to the user when operating the robot if the terminal is not visible by the camera operator. This function can deal with invalid input arguments and will only execute if the integer of the input quantity is greater than 0. An error message will be displayed for invalid input.

```python
def shakeHand(quantity):
	quantity = int(quantity)

	#Check if the quanitty value is valid, if not print error message
	if (quantity >=1):
		for i in range (quantity):
			goToPose(oOffRots[0],oOffRots[1],oOffRots[2]-shakeAngle,oOffCords[0],oOffCords[1],oOffCords[2])
			goToPose(oOffRots[0],oOffRots[1],oOffRots[2]+shakeAngle,oOffCords[0],oOffCords[1],oOffCords[2])
			goToPose(oOffRots[0],oOffRots[1],oOffRots[2],oOffCords[0],oOffCords[1],oOffCords[2])
	else:
		print("Shake not executed. Shake quantity value needs to be a positive integer")
```

&nbsp;

###### goOnPath Function
**********
This function takes a frame of coordinates, as it's input argument, and then executes the drawing of the frame by calculating a motion plan and executing it on the robot. This function works similarly to the goToPose function, however, group.compute_cartesian_path is used to calculate the motion plan as the trajectory is based upon multiple coordinates. The coordinates that make up a frame are converted into drawpose values inside the for loop and these are then added to the list of poses called drawpoints. The motion plan is calculated based on this list of robot poses.

```python
def goOnPath(frame):
	#Clear previous pose targets
	group.clear_pose_targets()

	#Create an empty list to store the frame coordinates
	drawpoints = []

	for coordinate in frame:
		#Calculate the quaternion of the end effector facing forward
		quaternion = tf.transformations.quaternion_from_euler(oOffRots[0],oOffRots[1],oOffRots[2])

		#Initialise the drawpose message 
		drawpose = geometry_msgs.msg.Pose()

		#update the drawpose message base upon the required rotation and coordinate location of the end effector
		drawpose.orientation.x = quaternion[0]
		drawpose.orientation.y = quaternion[1]
		drawpose.orientation.z = quaternion[2]
		drawpose.orientation.w = quaternion[3]
		drawpose.position.x = oOffCords[0] + coordinate[0] *frameScale
		drawpose.position.y = oOffCords[1] + coordinate[1] *frameScale
		drawpose.position.z = oOffCords[2] + coordinate[2] *frameScale

		#Add the drawpose message to the list of drawpoints
		drawpoints.append(copy.deepcopy(drawpose))

	#Calculate motion path base on drawpoints list
	(plan, fraction) = group.compute_cartesian_path(drawpoints, 0.01, 0.0)

	#Execute the calculated motion plan and wait until complete
	group.execute(plan, wait=True) 
```

&nbsp;

###### lightOn Function
**********
This function will turn on the light attached to the end effector by closing the gripper based upon the value set in the lightOnDist variable.
```python
def lightOn():
	gripper_msg.data = [lightOnDist, lightOnDist]
	gripper_publisher.publish(gripper_msg)	
```

&nbsp;

###### lightOff Function
**********
This function will turn off the light attached to the end effector by opening the gripper based upon the value set in the lightOffDist variable.
```python
def lightOff():
	gripper_msg.data = [lightOffDist, lightOffDist]
	gripper_publisher.publish(gripper_msg)
```


&nbsp;

&nbsp;

#### Storing Animation (frames)
_____
The animation frames are stored in the framelist in the python file frames. This is the output data from the animation creation process in Rhino. Each frame is comprised of a list of 3 value lists which contain a coordinate value [x,y,z]. Multiple frames make up an animation sequence. An example of one frame from the butterfly animation can be seen below.

```python
framelist = [[[0, 0, 0],
[-30.112995, 0, 4.267024],
[-29.869165, 0, -5.730003],
[0, 0, 0],
[19.752333, 0, 5.902996],
[20.161001, 0, -3.087721],
[0, 0, 0],
[-26.639191, -9.011853, -26.175182],
[26.034624, -20.905941, -26.17518],
[0, 0, 0],
[-26.639191, 9.011853, -26.175182],
[26.034624, 20.905941, -26.17518],
[0, 0, 0.1]]]
```