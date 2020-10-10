import rclpy
from rclpy.node import Node
#import json
from robot_msgs.msg import RobotSpeed
import urllib.request


'''     
while 1:
        url = urllib.request.urlopen('http://35.236.229.125/key_parser')
        data = str(url.read())
        print(data[-9:-7])
        sleep(1)
'''

actions = {
        'w': [100,100, 50],
        'a': [100,15,50],
        'd': [15,100,50],
        's': [0,0,0]
        }

class RobotTeleopPublisher(Node):

    def __init__(self):
        super().__init__('robot_teleop_publisher')
        self.publisher_ = self.create_publisher(RobotSpeed, 'robot_speed', 1)
        print("Starteing to recieve messages!!")
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = RobotSpeed()
        url = urllib.request.urlopen('http://35.236.229.125/key_parser')
        data = str(url.read())
        action = [0,0,0]
        #act = json.loads(data[0:-3]) 
        #print(data[15:17])
        try:
            #print(data[15:17]=='87')
            if(data[15:17]=='87'):
                print('w')
                action = actions['w']
            elif (data[15:17]=='65'):
                print('a')
                action = actions['a']
            elif (data[15:17]=='68'):
                print('d')
                action = actions['d']
        except:
            pass

        msg.lspeed = action[1]
        msg.rspeed = action[0]
        msg.steps = action[2]


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

