# agv-arduino-keyboard
Arduino Due code for controlling AGV using teleop-twist-keyboard ROS node.

System Specification:

Arduino Due

ROS Melodic

Ubuntu 18.04

Guide:

Step 1: Upload this code into Arduino Due

Step 2: Start roscore

Step 3: Start rosserial (search for the guide how to install and use it)

Step 4: Start teleop_twist_keyboard

Code Brief Explanation:

The code converts x linear velocity (max 1m/s) and z angular velocity (max 2 rad/s) to inverted Arduino PWM (229 - 0). PWM 229 is 10% PWM that our motor driver requires for zero velocity. This code allows you to move forward, reverse and differential rotation at half the speed (PWM) using the keyboard. The speed can be adjusted using the keyboard ROS node.

I recommend for your to use wireless keyboard (that shares the same dongle with a mouse) to control the robot. Arduino Due must always connected to the ROS computer via USB at all times.

Any questions you can email me at zharif.z@prostrain.com.my. If no reply after 1 month, this means there will be no reply at all.

Goodluck and all the best!
