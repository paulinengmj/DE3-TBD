## Future Development
**********
The following section includes future development aspects that can be implemented to further develop this project.

####  Automating Image Capture

This could be implemented through designing a custom shutter with a large surface area connected to the camera. The shutter could then be pressed by the robot in-between frames to open and close the aperture before and at the end of the motion. Another option could be to use an infrared sensor which the robot would need to swipe over, this would require less advanced hardware and could prevent damage to robot due to collision. The robot operating sequence would in turn change with this setup, removing the need for a wiggle indication and shifting the priority towards turning the light on and off with the gripper preventing a streak in each frame. 


#### Image Compilation

This would be impossible using the current hardware available since the images are stored in an SD card on the camera. A way for the stop motion animation to be automatically compiled is by using a Raspberry Pi with a camera module. This can be implemented by appending each image to a sequence of images followed by a command to export it as a video. This would, therefore, allow the images to be automatically compiled as an animation at the end of the robot operating sequence.

#### Speed Control

 The intensity of the light in an image is proportional to the time spent in each position ie. speed, ```s```; the brightness of the torch, ```bt```; and angle of motion relative to the orbiting camera angle, ```a```  which varies sinusoidally, from maximum when ```a=0```. Image brightness, ``` B = (k* bt + x* sin(a))/ s ``` Where ```k``` and ```x ``` are constants. Thus, setting B to a constant and defining the pose and position of the camera would mean that the speed could be calculated to maintain a constant brightness in the light trail. However, in this project, the images taken were clear despite not implementing speed control and thus was considered a redundancy. 
 
#### Perception and Object Detection

The mechanism of picking up the torch for this project was executed by feeding it to the robot arm at a precise location. Implementing perception will provide the robot with the ability to identify the environment by the addition of a sensor such as an RGB-D camera, a webcam or a laser scanner. This would also allow for the robot to identify the light module as a target for a ```pick``` operation




