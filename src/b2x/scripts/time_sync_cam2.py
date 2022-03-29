'''
 @Author: JoeyforJoy & ylheng
 @Date: 2022-03-25 15:10:15
 @LastEditTime: 2022-03-29 11:13:51
 @LastEditors: JoeyforJoy
 @Description: Transfer rosbag to synchronized image and pcd files.
 @Example: 
    # message should be broadcast first
    rosrun b2x time_sync_cam2.py ${img1_topic} ${img2_topic} --output_dir ${output_dir}
'''

import numpy as np
import rospy
import message_filters
from sensor_msgs.msg import Image, CompressedImage

import os
import sys
PARENT_DIR = os.path.join(os.path.abspath(os.path.dirname(__file__)), "..")
sys.path.append(PARENT_DIR)
from utils import *
import argparse

def parse_args():
    parser = argparse.ArgumentParser(description="Transfer rosbag to synchronized image and pcd files.")
    parser.add_argument("topic_img1", type=str, help = "the name of the image1 topic")
    parser.add_argument("topic_img2", type=str, help = "the name of the image2 topic")
    parser.add_argument("--output_dir", type=str, default="./data/synchronized", 
                        help = "the root directory of the output files")
    parser.add_argument("--img1_dir_label", type=str, default="image1", 
                        help = "the subdirectory name of output images")
    parser.add_argument("--img2_dir_label", type=str, default="image2", 
                        help = "the subdirectory name of output images")
    parser.add_argument("--tot", type=float, default=0.01, 
                        help = "the tolerence of time synchronization")
    return parser.parse_args()

class callBackClass:
    def __init__(self, output_dir, img1_subdir="image1", img2_subdir="image2",
                        img1_compressed = True, img2_compressed = True):
        self.output_dir = output_dir
        self.img1_dir = os.path.join(self.output_dir, img1_subdir)
        self.img2_dir = os.path.join(self.output_dir, img2_subdir)
        
        os.makedirs(self.output_dir, exist_ok=True)
        os.makedirs(self.img1_dir, exist_ok=True)
        os.makedirs(self.img2_dir, exist_ok=True)
        
        self.img1_compressed = img1_compressed
        self.img2_compressed = img2_compressed

        self.count = 0
        self.max_count = 1000000

    def __call__(self, img1_msg, img2_msg):
        frame_name = "%06d" % (self.count)
        # print("frame name: %s\ttimestampe: %s" % (frame_name, img1_msg.header.stamp))

        # transfer img1 msg 2 cv img
        dumpImageMsg(img1_msg, self.img1_dir, frame_name, compressed = self.img1_compressed)
        dumpImageMsg(img2_msg, self.img2_dir, frame_name, compressed = self.img2_compressed)

        self.count = (self.count + 1) % self.max_count

if __name__ == "__main__":
    rospy.init_node('time_sync_lidar_cam')

    args = parse_args()

    image1_sub = createImgMsgFilterSubsciber(args.topic_img1)
    image2_sub = createImgMsgFilterSubsciber(args.topic_img2)
    ts = message_filters.ApproximateTimeSynchronizer([image1_sub, image2_sub], 10, args.tot, allow_headerless=True)

    img1_compressed = isCompressedImage(args.topic_img1)
    img2_compressed = isCompressedImage(args.topic_img2)
    callback = callBackClass(args.output_dir, img1_compressed = img1_compressed, img2_compressed = img2_compressed)
    ts.registerCallback(callback)
    rospy.spin()
