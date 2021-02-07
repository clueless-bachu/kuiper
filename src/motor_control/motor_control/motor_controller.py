'''
Author: Vasista Ayyagari
File Description: A helper file with Classes for Motor control to drive the robot
Copyright: Vasista Ayyagari, 2021
'''

# importing required libraries
import RPi.GPIO as gpio
import numpy as np
from time import sleep


class Motor():
    '''
    A class abstraciton of an individual motor being controlled using PWM. 
    '''
    def __init__(self, pins, pwmFreq = 100):
        '''
        Initializes the Motor and primes it to be used by a Driving system

        Inputs:
        pins: A pair of Raspberry Pi pins are required in order
        pwmFreq: The frquency at which PWM signal is set
        		
        Return: None
        '''
        assert len(pins)== 3, "Shape mismatch of the pins"
        gpio.setup(pins[0], gpio.OUT)
        gpio.setup(pins[1], gpio.OUT)
        gpio.setup(pins[2], gpio.OUT)

        gpio.output(pins[2], True)
        self.pwmFreq = pwmFreq
        self.pwm = [gpio.PWM(pins[0], pwmFreq),  gpio.PWM(pins[1], pwmFreq)]
        self.pwm[0].start(0)
        self.pwm[1].start(0)
        self.pwmdc = np.zeros(2)

    def setSpeed(self, speed):
        '''
      	Sets the PWM value and hence setting a particular speed

        Inputs:

         speed: A percentage value of the PWM signal strength. It if the range 0-100

        Return: None
        '''
        assert isinstance(speed, float) or isinstance(speed, int) , "Speed is not a number"
        if(speed>0):
            pwmdc = np.array([speed,0])
        else:
            pwmdc = np.array([0, abs(speed)])

        self.pwm[0].ChangeDutyCycle(pwmdc[0])
        self.pwm[1].ChangeDutyCycle(pwmdc[1])
        self.pwmdc = pwmdc

    def getSpeed(self):
        '''
        Initializes the Motor and primes it to be used by a Driving system

        Inputs: None

        Return: 
        A numpy array denoting the PWM values set on the pins of the motor
        '''
        if(np.argmax(self.pwmdc)):
            return -self.pwmdc[1]
        else:
            return self.pwmdc[0]


class MotorSystem():
    def __init__(self, pins):
        '''
	A class abstraction of the entire system of motors. The commands can be move front,left
	etc. Controls all the motors at once
        Convention: While looking from the rear, the front wheel on the right is fr

        Inputs:

        pins: A dictionary wit keys as motor name and values as a pair of
        Raspberry Pi pins are required in order

        Return: None
        '''
        super(MotorSystem, self).__init__()                
        assert isinstance(pins, dict) and len(pins.keys()) ==4,"The pins argument is incorrect, its length should be 4"
        self.motor = {}
        gpio.setmode(gpio.BCM)
        self.motor['fr'] = Motor(pins['fr'])
        self.motor['fl'] = Motor(pins['fl'])
        self.motor['br'] = Motor(pins['br'])
        self.motor['bl'] = Motor(pins['bl'])

        self.curSpeeds = np.zeros(2)

    def generateSpeed(self, speed, steps):
        '''
        Returns an iterator to slowly set the speed of the robot to a particular value.
	It interpolates intermediate speed vales 

        Inputs:

        speed: The final speed the robot needs to reach, represented as [rightSpeed, leftSpeed]
        steps: Number of steps taken before reaching final speed

        Return: 
         An iterator that generates the next speed
        '''
        inc = (speed- self.curSpeeds)/steps
        for i in range(1, steps+1):
            yield self.curSpeeds + i*inc

    def setSpeed(self, speed, mindiff = 3):
        '''
	Sets the Speed of the robotic system
        
        Inputs:

         speed: The final speed the robot needs to reach, represented as [rightSpeed, leftSpeed]
         steps: Number of steps taken before reaching final speed

        Return: 
         None
        '''
        assert speed.shape == (2,), "The shape of speed variable is incorrect, it should be (2,)"
       
        interSpeeds = speed - self.curSpeeds
        mindir = np.sign(interSpeeds)

        veldiff = np.zeros(2)

        if abs(interSpeeds[0])>mindiff: veldiff[0] = mindir[0]*mindiff
        else: veldiff[0] = interSpeeds[0]

        if abs(interSpeeds[1])>mindiff: veldiff[1] = mindir[1]*mindiff
        else: veldiff[1] = interSpeeds[1]


        self.motor['fr'].setSpeed(self.curSpeeds[0]+veldiff[0])
        self.motor['fl'].setSpeed(self.curSpeeds[1]+veldiff[1])
        self.motor['br'].setSpeed(self.curSpeeds[0]+veldiff[0])
        self.motor['bl'].setSpeed(self.curSpeeds[1]+veldiff[1])

        self.curSpeeds = self.curSpeeds + veldiff

    def brake(self):
        '''
	Applies emmergency brakes to the robot

        Inputs:
	 None

        Return: 
         None
        '''
        self.motor['fr'].setSpeed(0)
        self.motor['fl'].setSpeed(0)
        self.motor['br'].setSpeed(0)
        self.motor['bl'].setSpeed(0)
        self.curSpeeds = np.zeros(2)


    def getSpeed(self):
        '''
	Returns the speed of the robot
        Inputs:
	 None

        Return: 
        Speed of the robot
        		'''
        return self.curSpeeds


    def stop(self):
        '''
	Shuts down the PWM port of all the pins of the robot

        Inputs:
	 None

        Return: 
         None
        '''
        self.motor['fr'].pwm[0].stop()
        self.motor['fr'].pwm[1].stop()

        self.motor['fl'].pwm[0].stop()
        self.motor['fl'].pwm[1].stop()

        self.motor['br'].pwm[0].stop()
        self.motor['br'].pwm[1].stop()

        self.motor['bl'].pwm[0].stop()
        self.motor['bl'].pwm[1].stop()

    def __del__(self):
        '''
	Cusstom Object delete function that properly shuts down the robot system after braking

        Inputs:
	 None

        Return: 
         None
        '''
        self.brake()
        self.stop()
        gpio.cleanup()
        print("All gpio pins are disconnected")


