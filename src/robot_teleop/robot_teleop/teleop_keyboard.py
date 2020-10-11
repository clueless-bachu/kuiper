# Import libraries
import rclpy
from rclpy.node import Node
from robot_msgs.msg import RobotSpeed
import sys, select, termios, tty

settings = termios.tcgetattr(sys.stdin)
def getKey():
    '''
    retrieves the key pressed on the keyboard. Courtesy Willow Garage.
    Inputs:
         None

    Return: 
     None
    '''
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

mxjump = 10
actions = {
        'w': [100,100, mxjump],
        'a': [100,15,mxjump],
        'd': [15,100,mxjump],
        's': [0,0,100],
        'x': [-100, -100, mxjump],
        'z': [-100,-15,mxjump],
        'c': [-15, -100, mxjump]
        }

class RobotTeleopPublisher(Node):
    '''
    A ROS2 Node for publishing speed data which is accessed using the Keyboard
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
        # interval time at which call back function is called
        timer_period = 0.00001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) 

    def timer_callback(self):
        '''
        Callback function. This accesses the keyboard using getKey() for speed commands and
        publishes to the robot_speed topic
         Inputs:
         None

        Return: 
         None
        '''
        msg = RobotSpeed()
        key = getKey()

        if key == 'p': exit(0)
        try:
           msg.rspeed = actions[key][0]
           msg.lspeed = actions[key][1]
           msg.steps = actions[key][2]
        except:
           msg.rspeed = 0
           msg.lspeed = 0
           msg.steps = 20

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
