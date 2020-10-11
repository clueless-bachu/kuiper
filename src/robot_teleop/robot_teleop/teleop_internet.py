# Import libraries
import rclpy
from rclpy.node import Node
from robot_msgs.msg import RobotSpeed
import urllib.request

# Define the set of actions
actions = {
        '87' [100,100, 3],
        '65': [100,15,3],
        '68': [15,100,3],
        '83': [-100,-100,3]
        }

class RobotTeleopPublisher(Node):
    '''
    A ROS2 Node for publishing speed data which is accessed using the Kuiper API 
    '''
    def __init__(self):
        '''
        Initializes the publisher
        Inputs:
         None

        Return: 
         None
        '''
        super().__init__('robot_teleop_publisher')
        self.publisher_ = self.create_publisher(RobotSpeed, 'robot_speed', 1)
        print("Starteing to recieve messages!!")
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

        verbose = True
        msg = RobotSpeed()
        url = urllib.request.urlopen('http://35.236.229.125/key_parser')
        data = str(url.read())
        action = [0,0,0]
        
        try:
            action = actions[data[15:17]]
            print(data[15:17])
        except:
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
    robot_teleop_publisher = RobotTeleopPublisher()

    rclpy.spin(robot_teleop_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_teleop_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

