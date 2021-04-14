'''
Author: Vasista Ayyagari
File Description: Reads the motion commands from the Kuiper API and publishes it to the ROS network
Copyright: Vasista Ayyagari, 2021
'''
# Import libraries
import rclpy
from rclpy.node import Node
from robot_msgs.msg import RobotSpeed
import urllib.request
import requests

# Define the set of actions
actions = {
        '87': [100,94, 3],
        '65': [100,15,3],
        '68': [15,100,3],
        '83': [0,0,100],
        '88': [-100, -87, 3],
        '90': [-100, -15, 3],
        '67': [-15, -100, 3]
        }

class RobotTeleopPublisher(Node):
    '''
    A ROS2 Node for publishing speed data which is accessed using the Kuiper API 
    '''
    def __init__(self, url, route):
        '''
        Initializes the publisher
        Inputs:
         None

        Return: 
         None
        '''
        super().__init__('robot_teleop_publisher')
        self.publisher_ = self.create_publisher(RobotSpeed, 'robot_speed', 1)

        self.url = "http://"+url+route
        print("Starting to recieve messages!!")
        # interval time at which call back function is called
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        '''
        Callback function. This accesses the kuiper API for speed commands and publishes to the 
        robot_speed topic
         Inputs:
         None

        Return: 
         None
        '''

        msg = RobotSpeed()
#        url = urllib.request.urlopen(self.url)
        r = requests.get(url = self.url)
        data = str(r.json()['response'])
        action = [0,0,3]
        
        try:
            action = actions[data]
        except Exception as e:
            pass


        # Setting speed value in the message
        msg.lspeed = action[1]
        msg.rspeed = action[0]
        msg.steps = action[2]


        self.publisher_.publish(msg)


def main(args=None):
    # Intializes Node
    rclpy.init(args=args)
    # creates publisher class
    url = "192.168.43.16:5000/"
    route = "key_parser"
    robot_teleop_publisher = RobotTeleopPublisher(url, route)

    rclpy.spin(robot_teleop_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_teleop_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

