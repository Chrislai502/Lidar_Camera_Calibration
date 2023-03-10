import sys
import os
import sensor_msgs.msg as sensor_msgs
import struct
import ctypes
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt
import rclpy
from rclpy.clock import ROSClock
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
# import open3d as o3d
import numpy as np
# Own files
import params 
from pcd_image_overlay_Chris import create_point_cloud_image_overlay
import pointcloud2_to_pcd_file  

# from tf2_ros import TransformException
# from tf2_ros.buffer import Buffer
# from tf2_ros.transform_listener import TransformListener

# import time

from scipy.spatial.transform import Rotation as R

# ---------------------------------------------------------------------------- #
# -------------- Subscribes to the image and ptc, then publishes ------------- #
# ----------------------- the overlay as an Image topic ---------------------- #
# ---------------------------------------------------------------------------- #
class Img_PCD_Subscriber_Overlay_Publisher(Node):

    # Initializing buffers
    glob_cv_image_front = None
    ros_image_point_cloud_overlay = None

    # Experimenting
    glob_pcd_file = None
    glob_pcd_file2 = None


    def __init__(self):
        super().__init__('img_pcd_subscriber_overlay_publisher')

        # ------------------------------ Chris Add Start ----------------------------- #
        # -------------------------------- QOS Profile ------------------------------- #
        self.qos_profile =  QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # ------------------------------- Chris Add End ------------------------------ #
        # -------------------------------- Subscribers ------------------------------- #
        # Subscribe to pcd file
        self.pcd_file = self.create_subscription(
            sensor_msgs.PointCloud2,
            params.sub_topic_pcd,  # Subscribes from pcd_publisher 
            self.sub_callback_pcd,
            self.qos_profile)
        self.pcd_file  # Prevent unused variable warning

        # Subscribe to image file
        self.ros_img_front = self.create_subscription(
            sensor_msgs.Image,
            params.sub_topic_image,  # Subscribes from image publisher
            self.sub_callback_img,
            self.qos_profile)
        self.ros_img_front  # Prevent unused variable warning

        # Publish overlay as an image
        self.overlay_publisher = self.create_publisher(
            sensor_msgs.Image,
            params.pub_topic_overlay,
            rclpy.qos.qos_profile_sensor_data)

        # Does one Overlay Projection every "params.overlay_publish_timestep" time 
        self.publish_timestep = params.overlay_publish_timestep 
        self.timer_overlay = self.create_timer(self.publish_timestep, self.pub_callback)    

        # Declaring CV bridge for converting ROS sensor Images to OpenCV Images
        self.bridge = CvBridge()

        # # subscribe to lookup transform and have initial transform written
        # # to params.py

        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer, self)

        # from_frame_rel = params.lidar_frame # From
        # to_frame_rel = params.camera_frame # To


        # print("Sleeping...")
        # # Sleep the code to wait for the transforms to be ready
        # time.sleep(3)
        # print("Awake!...")

        # got_transform = False
        # while not got_transform:
        #     print(self.tf_buffer)
        #     try:
        #         t = self.tf_buffer.lookup_transform(
        #             target_frame = to_frame_rel,
        #             source_frame = from_frame_rel,
        #             time = rclpy.time.Time())
        #         got_transform = True
        #         print("Got the Transform!")
        #     except TransformException as ex:
        #         self.get_logger().info(
        #             f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
        #         print("No transform was found")
        #         # return
        #     time.sleep(0.5)
        
        # print("Transforms Start here: \n", t)
        # print("Transforms End")
        

        # x = t.transform.translation.x
        # y = t.transform.translation.y
        # z = t.transform.translation.z
        # params.translation = (x, y, z)

        # x = t.transform.rotation.x
        # y = t.transform.rotation.y
        # z = t.transform.rotation.z
        # w = t.transform.rotation.w
        # r = R.from_quat([x, y, z, w])
        # r_ = r.as_euler('zyx', degrees=True)
        # params.rotation = (r[0], r[1], r[2])



    # -------------------- Image subscriber callback function -------------------- #
    def sub_callback_img(self, Image):
        print("i")
        Img_PCD_Subscriber_Overlay_Publisher.glob_cv_image_front = None # Clears out the buffer first
        try: 
            Img_PCD_Subscriber_Overlay_Publisher.glob_cv_image_front  = self.bridge.imgmsg_to_cv2(Image)
            
        except CvBridgeError as e:
            print(e)
        # print("Subscribed to image.") 

    # --------------- Pointcloud2 file subscriber callback function -------------- #
    def sub_callback_pcd(self, PointCloud2 ):
        print("p")
        # The 'msg', which is of the type PointCloud2 is converted to a pcd and finally to an array.
        # The function read_points2 is ported from the ROS1 package below. 
        # https://github.com/ros/common_msgs/blob/noetic-devel/sensor_msgs/src/sensor_msgs/point_cloud2.py
        Img_PCD_Subscriber_Overlay_Publisher.glob_pcd_file = None
        Img_PCD_Subscriber_Overlay_Publisher.glob_pcd_file2 = None
        gen2  = pointcloud2_to_pcd_file.read_points(PointCloud2, field_names=['x', 'y', 'z'], skip_nans=True) # returns a pointcloud Generator
        gen = pointcloud2_to_pcd_file.read_points(PointCloud2, skip_nans=True) # returns a pointcloud Generator
        
        Img_PCD_Subscriber_Overlay_Publisher.glob_pcd_file2 = np.array(list(gen2))
        Img_PCD_Subscriber_Overlay_Publisher.glob_pcd_file = np.array(list(gen))
        
        # ---------------------------------------------------------------------------- #
        #                                 Chris Method                                 #
        # ---------------------------------------------------------------------------- #


        # print("Subscribed to pcd.")


    # -------------------- Overlay publisher callback function (Timer Invoked) ------------------- #
    def pub_callback(self):
        cv_image_point_cloud_overlay = None # Clear the buffer
        # print("Img_PCD_Subscriber_Overlay_Publisher.glob_pcd_file==None:, ", Img_PCD_Subscriber_Overlay_Publisher.glob_pcd_file==None)
        # print(np.any(np.array([Img_PCD_Subscriber_Overlay_Publisher.glob_pcd_file==None, Img_PCD_Subscriber_Overlay_Publisher.glob_pcd_file2 == None, Img_PCD_Subscriber_Overlay_Publisher.glob_cv_image_front == None])))
        # ------------------------------ Check if Empty ------------------------------ #
        if (np.any([np.any(Img_PCD_Subscriber_Overlay_Publisher.glob_pcd_file2 == None), \
                    np.any(Img_PCD_Subscriber_Overlay_Publisher.glob_cv_image_front == None)])):
            print("Some Inputs are Empty")
            return

        # ---------------------------------------------------------------------------- #
        #                         Overlay image and pointcloud                         #
        # ---------------------------------------------------------------------------- #
        ros_image_point_cloud_overlay = create_point_cloud_image_overlay\
            (Img_PCD_Subscriber_Overlay_Publisher.glob_pcd_file2, Img_PCD_Subscriber_Overlay_Publisher.glob_cv_image_front)
        
        # Convert overlay to ros msg image format (using cv_bridge)
        try:
            ros_image_point_cloud_overlay = self.bridge.cv2_to_imgmsg(ros_image_point_cloud_overlay)
        except CvBridgeError as e:
            print(e) 
            return
        

        ros_image_point_cloud_overlay.header.frame_id = "camera_front_point_cloud_overlay"
        clock = ROSClock()
        timestamp = clock.now()
        ros_image_point_cloud_overlay.header.stamp = timestamp.to_msg()
        # print("Overlay published.")
        self.overlay_publisher.publish(ros_image_point_cloud_overlay)

def main(args = None):
    rclpy.init(args = args)
    img_pcd_sub_overlay_pub = Img_PCD_Subscriber_Overlay_Publisher()
    rclpy.spin(img_pcd_sub_overlay_pub)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
