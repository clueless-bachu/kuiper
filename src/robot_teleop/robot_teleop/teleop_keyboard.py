import rclpy
from rclpy.node import Node

from robot_msgs.msg import RobotSpeed
import sys, select, termios, tty

settings = termios.tcgetattr(sys.stdin)

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

actions = {
        'w': [100,100, 50],
        'a': [100,15,50],
        'd': [15,100,50],
#        's': [0,0,0]
        }



class RobotTeleopPublisher(Node):

    def __init__(self):
        super().__init__('robot_teleop_publisher')
        self.publisher_ = self.create_publisher(RobotSpeed, 'robot_speed', 1)
        timer_period = 0.00001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) 

    def timer_callback(self):
        msg = RobotSpeed()
        key = getKey()
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
    rclpy.init(args=args)

    robot_teleop_publisher = RobotTeleopPublisher()

    rclpy.spin(robot_teleop_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_teleop_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
