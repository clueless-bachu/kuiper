# Kuiper-Bot
A robot that can be controlled from anywhere in the world via the internet


## Installation
Download the entire repository
```
$ git clone https://github.com/clueless-bachu/Kuiper-Bot.git
```

Add the source files to your ROS2 workspace and build it
```
$ cd <name of workspace>
$ colcon build 
$ . install/setup.bash
```

You can optionally add the Kuiper API, a flask app to your web dev project or use it as a standalone app by hosting it using a server.

## Edit

Make changes to the IP address in ``` src/robot_teleop/robot_teleop/teleop_internet.py``` to the appropriate IP address of the hosted API

Additionally, make changes to ```kuiper-api/kuiper.py``` and change the login credentials to name

Change the pins in ```Kuiper-Bot\src\motor_control\motor_control\driving_system.py``` to custom Pins required for your robot

## Running

In you hosting platform, you can simply run ```kuiper.py```
```
$ sudo python kuiper.py
or
$ sudo python3 kuiper.py
```

To run the robot, open different terminals and run the following commands
```
Terminal 1
$ . install/setup.bash
$ ros2 run motor_control driver

Terminal 2
$ . install/setup.bash
$ ros2 run robot_teleop teleop_internet (This is to control the robot using the API)
or
$ ros2 run robot_teleop teleop_keyboard (This is to control the robot using the Keyboard)
```

## Teleop Controls

Whether you are controlling the robot using you keyboard on the terminal or using the kuiper API web app. The controls are the same. The motivation behind the controls is to "gamify" the teleoperation. This should be intuitive, easy to control with one hand. Most computer video games employ the use of w,a,s,d keys to move their main character. This package also uses the same.This idea is in contrast to the teleoperation control offered by willow garage in controlling a robot say the Turtlebot. In their implementation, they set speeds upon a key press which lingers even when the keys are released. To stop the robot, the operator has to press another button. In this implementation, the robot comes to a stop if key is released giving the feel of a racecar video game in real life. 

Here are the controls


w: Move forward

a: Turn Left

d: Turn Right

s: brakes

x: Move backward/reverse

z: Turn left while moving backwards

c: Turn right while moving backwards

## Robot Description
The robot is a 4 wheel drive controlled by two L298N motor driver driving two motors each. The motor driver is controlled by a single Raspberry Pi. The system is powered by a single 11.1V LiPo battery with a power distribution circuit and 12V to 5V buck converter to power the onboard computer.

## Additional Description
YouTube Link: https://www.youtube.com/watch?v=nv5OmhN1pKg:wq


Devpost Descrioption: https://devpost.com/software/kuiperbot
