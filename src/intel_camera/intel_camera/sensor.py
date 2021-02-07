'''
Author: Vasista Ayyagari
File Description: Creates a node that reads sensor data from Intel Realsense Camera and publishes it to the ROS network 
Copyright: Vasista Ayyagari, 2021
'''

# Import Libraries
import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import cv2
from cv_bridge import CvBridge
import numpy as np
from math import tan, pi
from time import time
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image
from threading import Lock


# Start another Thread for reading sensor data parallely
# This improves FPS by a huge margin
frame_mutex = Lock()
frame_data = {"left"  : None,
              "img_timestamp_ms": None,
              "pos_timestamp_ms" : None,
              "pos": None,
              "quat": None
              }


def get_extrinsics(src, dst):
    """
    Returns R, T transform from src to dst
    """
    extrinsics = src.get_extrinsics_to(dst)
    R = np.reshape(extrinsics.rotation, [3,3]).T
    T = np.array(extrinsics.translation)
    return (R, T)

def camera_matrix(intrinsics):
    """
    Returns a camera matrix K from librealsense intrinsics
    """
    return np.array([[intrinsics.fx,             0, intrinsics.ppx],
                     [            0, intrinsics.fy, intrinsics.ppy],
                     [            0,             0,              1]])

def fisheye_distortion(intrinsics):
    """
    Returns the fisheye distortion from librealsense intrinsics
    """
    return np.array(intrinsics.coeffs[:4])

def callback(frame):
    """
    This callback is called on a separate thread, so we must use a mutex
    to ensure that data is synchronized properly. We should also be
    careful not to do much work on this thread to avoid data backing up in the
    callback queue.
    """
    global frame_data
    
    if frame.is_frameset():
        frameset = frame.as_frameset()
        f1 = frameset.get_fisheye_frame(1).as_video_frame()
        left_data = np.asanyarray(f1.get_data())
        ts = frameset.get_timestamp()
        frame_mutex.acquire()
        frame_data["left"] = left_data
        frame_data["img_timestamp_ms"] = ts
        frame_mutex.release()
    elif frame.is_pose_frame():
        pose_frame = frame.as_pose_frame()
        data = pose_frame.get_pose_data()
        if data:
            position = data.translation
            quaternion = data.rotation
            ts = frame.get_timestamp()
            frame_mutex.acquire()
            frame_data["pos"] = position
            frame_data["quat"] = quaternion
            frame_data["pos_timestamp_ms"] = ts
            frame_mutex.release()

class SensorPublisher(Node):
    '''
    A Publisher class that reads senor data and publisher postion and Img data 
    '''
    def __init__(self):
        '''
        Initializes the ROS2 node
        Inputs: None
        Return: None
        '''
        super().__init__('sensor_publisher')
        self.pose_pub = self.create_publisher(Pose2D, 'pose2D_robot', 10)
        self.img_pub = self.create_publisher(Image, 'cam_image_robot', 1)
        pose_timer_period = 0.5 
        img_timer_period = 0.1
        self.pose_timer = self.create_timer(pose_timer_period, self.pose_timer_callback)
        self.img_timer = self.create_timer(img_timer_period, self.img_timer_callback)

        # Publishers created, now creating pipeline
        self.pipe = rs.pipeline()
        self.cfg = rs.config()
        self.pipe.start(self.cfg, callback)

        # Pipeline Started, intialising data for pose2D
        self.pos = None
        self.pos_flag = False

        # Initialising data for Img 

        window_size = 5
        min_disp = 0
        # must be divisible by 16
        num_disp = 112 - min_disp
        max_disp = min_disp + num_disp

        # Retreive the stream and intrinsic properties for both cameras
        profiles = self.pipe.get_active_profile()
        streams = {"left"  : profiles.get_stream(rs.stream.fisheye, 1).as_video_stream_profile(),
                   }
        intrinsics = {"left"  : streams["left"].get_intrinsics(),
                      }

        K_left  = camera_matrix(intrinsics["left"])
        D_left  = fisheye_distortion(intrinsics["left"])
        stereo_fov_rad = 90 * (pi/180)  # 90 degree desired fov
        stereo_height_px = 300          # 300x300 pixel stereo output
        stereo_focal_px = stereo_height_px/2 / tan(stereo_fov_rad/2)

        R_left = np.eye(3)

        stereo_width_px = stereo_height_px + max_disp
        stereo_size = (stereo_width_px, stereo_height_px)
        stereo_cx = (stereo_height_px - 1)/2 + max_disp
        stereo_cy = (stereo_height_px - 1)/2

        P_left = np.array([[stereo_focal_px, 0, stereo_cx, 0],
                                [0, stereo_focal_px, stereo_cy, 0],
                                [0,               0,         1, 0]])
        m1type = cv2.CV_32FC1
        (lm1, lm2) = cv2.fisheye.initUndistortRectifyMap(K_left, D_left, R_left, P_left, stereo_size, m1type)
        self.undistort_rectify = {"left"  : (lm1, lm2)}
        self.bridge = CvBridge()
        print("Camera is ready !!")


    def pose_timer_callback(self):
        '''
        A callback function to regularly publish the position data

        Input:
        None

        Return:
        None
        '''
        frame_mutex.acquire()
        valid = frame_data["pos_timestamp_ms"] is not None
        frame_mutex.release()
        if valid:
            frame_mutex.acquire()
            frame_copy = {"pos"  : frame_data["pos"],
                          "quat" : frame_data["quat"] 
                          }
            frame_mutex.release()
            if frame_data["pos"] != None:
                self.pos = frame_data["pos"]
                self.pos_flag = True
    
        if self.pos_flag:
            msg = Pose2D()
            msg.x = self.pos.x
            msg.y = self.pos.y
            msg.theta = 0.69
            self.pose_pub.publish(msg)


    def img_timer_callback(self):
        '''
        A callback function to publish the Image data

        Input:
        None

        Return:
        None
        '''
        frame_mutex.acquire()
        valid = frame_data["img_timestamp_ms"] is not None
        frame_mutex.release()
        if valid:
            frame_mutex.acquire()
            frame_copy =  {"left"  : frame_data["left"].copy(),
                          }
            frame_mutex.release()

            center_undistorted = {"left" : cv2.remap(src = frame_copy["left"],
                                              map1 = self.undistort_rectify["left"][0],
                                              map2 = self.undistort_rectify["left"][1],
                                              interpolation = cv2.INTER_LINEAR),
                                        }
            color_image = center_undistorted['left'][:,:]
            msg = self.bridge.cv2_to_imgmsg(color_image, encoding="passthrough")
            #msg.data = color_image
            self.img_pub.publish(msg)

    def __del__(self):
        print("-----------Stopping the Camera Sensor----------")
        self.pipe.stop()


def main(args=None):
    rclpy.init(args=args)

    sensor_publisher = SensorPublisher()

    rclpy.spin(sensor_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sensor_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
