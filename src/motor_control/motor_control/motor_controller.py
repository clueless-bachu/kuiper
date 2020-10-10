import RPi.GPIO as gpio
import numpy as np
from time import sleep


class Motor():
        def __init__(self, pins, pwmFreq = 100):
                assert len(pins)== 3, "Shape mismatch of the pins"
                gpio.setup(pins[0], gpio.OUT)
                gpio.setup(pins[1], gpio.OUT)
                gpio.setup(pins[2], gpio.OUT)

                gpio.output(pins[2], True)
                self.pwmFreq = pwmFreq
                self.pwm = [gpio.PWM(pins[1], pwmFreq),  gpio.PWM(pins[2], pwmFreq)]
                self.pwm[0].start(0)
                self.pwm[1].start(0)
                self.pwmdc = np.zeros(2)


        def setSpeed(self, speed):

                assert isinstance(speed, float) or isinstance(speed, int) , "Speed is not a number"
                if(speed>0):
                        pwmdc = np.array([speed,0])
                else:
                        pwmdc = np.array([0, abs(speed)])

                self.pwm[0].ChangeDutyCycle(pwmdc[0])
                self.pwm[1].ChangeDutyCycle(pwmdc[1])
                self.pwmdc = pwmdc

        def getSpeed(self):
                if(np.argmax(self.pwmdc)):
                        return -self.pwmdc[1]
                else:
                    return self.pwmdc[0]


class MotorSystem():
        def __init__(self, pins):
                
#               Convention: While looking from the rear, the front wheel on the right is fr
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
                inc = (speed- self.curSpeeds)/steps
                for i in range(1, steps+1):
                        yield self.curSpeeds + i*inc

        def setSpeed(self, speed, steps = 100):
                
                assert speed.shape == (2,), "The shape of speed variable is incorrect, it should be (2,)"

                interSpeeds = speed- self.curSpeeds#self.generateSpeed(speed, steps)

                self.motor['fr'].setSpeed(self.curSpeeds[0]+min(10, interSpeeds[0]))
                self.motor['fl'].setSpeed(self.curSpeeds[1]+min(10, interSpeeds[1]))
                self.motor['br'].setSpeed(self.curSpeeds[0]+min(10, interSpeeds[0]))
                self.motor['bl'].setSpeed(self.curSpeeds[1]+min(10, interSpeeds[1]))

                self.curSpeeds = self.curSpeeds + np.array([min(10, interSpeeds[0]),min(10, interSpeeds[1])])

        def brake(self):
                self.motor['fr'].setSpeed(0)
                self.motor['fl'].setSpeed(0)
                self.motor['br'].setSpeed(0)
                self.motor['bl'].setSpeed(0)
                self.curSpeeds = np.zeros(2)


        def getSpeed(self):
                return self.curSpeeds


        def stop(self):
                self.motor['fr'].pwm[0].stop()
                self.motor['fr'].pwm[1].stop()

                self.motor['fl'].pwm[0].stop()
                self.motor['fl'].pwm[1].stop()

                self.motor['br'].pwm[0].stop()
                self.motor['br'].pwm[1].stop()

                self.motor['bl'].pwm[0].stop()
                self.motor['bl'].pwm[1].stop()

        def __del__(self):
                self.brake()
                self.stop()
                gpio.cleanup()
                print("All gpio pins are disconnected")


