'''
 @Author: JoeyforJoy & ylheng
 @Date: 2022-03-25 15:10:15
 @LastEditTime: 2022-03-25 22:05:08
 @LastEditors: JoeyforJoy
 @Description: Transfer rosbag to synchronized image and pcd files.
 @Example: rosrun b2x time_sync_lidar_cam.py ${img_topic} ${pcd_topic} --output_dir ${output_dir}
'''

import numpy as np
import rospy
import message_filters
from sensor_msgs.msg import PointCloud2, CompressedImage
import cv2
from cv_bridge import CvBridge

import sensor_msgs.point_cloud2 as pc2

import os
import sys
PARENT_DIR = os.path.join(os.path.abspath(os.path.dirname(__file__)), "..")
sys.path.append(PARENT_DIR)
from utils import *
import argparse

def parse_args():
    parser = argparse.ArgumentParser(description="Transfer rosbag to synchronized image and pcd files.")
    parser.add_argument("topic_img", type=str, help = "the name of the image topic")
    parser.add_argument("topic_lidar", type=str, help = "the name of the pointcloud topic")
    parser.add_argument("--output_dir", type=str, default="./data/synchronized", 
                        help = "the root directory of the output files")
    parser.add_argument("--pcd_dir_label", type=str, default="pcd", 
                        help = "the subdirectory name of output pcds")
    parser.add_argument("--img_dir_label", type=str, default="image", 
                        help = "the subdirectory name of output images")
    parser.add_argument("--tot", type=float, default=0.01, 
                        help = "the tolerence of time synchronization")
    return parser.parse_args()

class callBackClass:
    def __init__(self, output_dir, pcd_subdir="pcd", img_subdir="image"):
        self.bridge = CvBridge()
        self.output_dir = output_dir
        self.pcd_dir = os.path.join(self.output_dir, pcd_subdir)
        self.img_dir = os.path.join(self.output_dir, img_subdir)
        
        os.makedirs(self.output_dir, exist_ok=True)
        os.makedirs(self.pcd_dir, exist_ok=True)
        os.makedirs(self.img_dir, exist_ok=True)
        
        self.count = 0
        self.max_count = 1000000

    def __call__(self, img_msg, lidar_msg):
        frame_name = "%06d" % (self.count)

        # transfer img msg 2 cv img
        cv_image = self.bridge.compressed_imgmsg_to_cv2(img_msg, "bgr8")
        img_path = os.path.join(self.img_dir, frame_name + ".png")
        cv2.imwrite(img_path, cv_image)

        # transfer PointCloud2 to pcd
        points = list(pc2.read_points(lidar_msg, skip_nans=True))
        num_points = len(points)
        points = np.array(points).reshape([num_points, -1])
        points[:, 3] /= 255 # normalize
        
        pcd_path = os.path.join(self.pcd_dir, frame_name + ".pcd")
        dumpAsPCD(pcd_path, points)

        self.count = (self.count + 1) % self.max_count

if __name__ == "__main__":
    rospy.init_node('time_sync_lidar_cam')

    args = parse_args()

    image_sub = message_filters.Subscriber(args.topic_img, CompressedImage)
    lidar_sub = message_filters.Subscriber(args.topic_lidar, PointCloud2)

    ts = message_filters.ApproximateTimeSynchronizer([image_sub, lidar_sub], 10, args.tot, allow_headerless=True)

    callback = callBackClass(args.output_dir)
    ts.registerCallback(callback)
    rospy.spin()
