# Kuiper-Bot
A robot that can be controlled from anywhere in the world via the internet


## Download and Installation of Source files in the ROS system
Download the entire repository, no need to make a complete workspace. This is a workspace on its own. build the files and source it
```
$ git clone https://github.com/clueless-bachu/Kuiper-Bot.git
$ colcon build 
$ . install/setup.bash
```

You can optionally add the Kuiper API, a flask app to your web dev project or use it as a standalone app by hosting it using a server.

## Download the API into your hosting platform
Download the kuiper-api to use it in your hosting platform
```
$ git clone https://github.com/clueless-bachu/Kuiper-Bot.git
```

## Edit

Make changes to the IP address in ``` src/robot_teleop/robot_teleop/teleop_internet.py``` and ``` src/intel_camera/intel_camera/post_img.py```  to the appropriate IP address of the hosted API. Finally make changes to the IP address mentioned in ```kuiper-api/static/js/keyProcess.js``` to the IP address of your server

Additionally, make changes to ```kuiper-api/kuiper.py``` and change the login credentials to name

Change the pins in ```Kuiper-Bot\src\motor_control\motor_control\driving_system.py``` to custom Pins required for your robot

## Running

In you hosting platform, you can simply run ```kuiper.py```
```
$ cd kuiper/kuiper-api
$ sudo python kuiper.py
or
$ sudo python3 kuiper.py
```

To run the robot, in a terminal run the following launch file. This will call all the nodes required to communicate with the kuiper-api and control the robot remotely
```
$ ros2 launch robot_teleop remote_teleop.py
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
YouTube Link: https://www.youtube.com/watch?v=IYt_oSYRyNA


Devpost Descrioption: https://devpost.com/software/kuiperbot
