'''
Author: Vasista Ayyagari
File Description: Subscribes to Image data and visualises it
Copyright: Vasista Ayyagari, 2021
'''

# Import libraries
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2

class TrackCamImgSub(Node):
    '''
    Simple subscriber node to visualise the Image data

    Input:
    None

    Return:
    None
    '''
    def __init__(self):
        super().__init__('track_cam_img_sub')
        self.subscription = self.create_subscription(
            Image,
            '/cam_image_robot',
            self.img_view_callback,
            2)

        self.bridge = CvBridge()
        self.subscription
        print("Viewer Node Initialised")

    def img_view_callback(self, msg):
        '''
        Subscriber callback that recievs Image data and visualises it

        Inpit: 
        msg: ROS2 message

        Returns:
        None
        '''
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "8UC1")
            cv2.imshow("Intel Left Camera View", cv_image)
            cv2.waitKey(3)
        except CvBridgeError as e:
            print(e)



def main(args = None):
    rclpy.init(args = args)

    trackCamImgSub = TrackCamImgSub()

    rclpy.spin(trackCamImgSub)

    trackCamImgSub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
