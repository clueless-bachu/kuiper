'''
Author: Vasista Ayyagari
File Description: Subscribes to Image data and posts it to the Kuiper website API 
copyright: Vasista Ayyagari, 2021
'''

# Import libraries
import rclpy
import requests 
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2
from time import sleep


class ImgPostSub(Node):
    '''
    subscriber node which posts the Image data to the Kuiper web API
    Input:
    None
    Return:
    None
    '''
    def __init__(self, url, route):
        super().__init__('img_post_sub')
        self.subscription = self.create_subscription(
            Image,
            '/cam_image_robot',
            self.img_post_callback,
            2)

        self.url = "http://"+url+route

        self.bridge = CvBridge()
        self.subscription
        print("Image Post Node Initialised")

    def img_post_callback(self, msg):
        '''
        Subscriber callback that recievs Image data and visualises it
        Inpit:
        msg: ROS2 message
        Returns:
        None
        '''
        sleep(0.01)
        try:
            cam_image = self.bridge.imgmsg_to_cv2(msg, "8UC1")
#            cam_image = cv2.resize(cam_image, (100,100))
            buffer = cv2.imencode('.jpg', cam_image)[1].T
            r = requests.post(url = self.url, data = buffer.tostring())
        except CvBridgeError as e:
            print(e)
        except Exception as e:
            print(e)


def main(args = None):
    rclpy.init(args = args)
    url = "35.245.143.252:80/"
    route = "compressed_image"
    imgPostSub = ImgPostSub(url, route)

    rclpy.spin(imgPostSub)

    imgPostSub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
