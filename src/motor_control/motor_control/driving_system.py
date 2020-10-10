
import rclpy
from rclpy.node import Node
import RPi.GPIO as gpio
import numpy as np
from robot_msgs.msg import RobotSpeed
from .motor_controller import *

class RobotSpeedSubscriber(Node):

    def __init__(self):
        super().__init__('robot_speed__subscriber')

        pins = {}

        pins['fr'] =  [25, 24, 18]
        pins['fl'] =  [27, 22, 4]
        pins['br'] =  [12, 20, 21]
        pins['bl'] =  [6, 5, 26]

        
        self.driver = MotorSystem(pins)

        self.subscription = self.create_subscription(
            RobotSpeed,
            'robot_speed',
            self.listener_callback,
            1)

        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        print(msg.lspeed, msg.rspeed, msg.steps)
        self.driver.setSpeed(np.array([msg.rspeed, msg.lspeed]), msg.steps)


def main(args=None):
    
    rclpy.init(args=args)

    robot_speed_subscriber = RobotSpeedSubscriber()
    print("Node will start accepting robot speeds")
    rclpy.spin(robot_speed_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_speed_subscriber.destroy_node()
    rclpy.shutdown()

    
if __name__ == '__main__':
    main()
