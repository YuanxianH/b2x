'''
 @Author: JoeyforJoy & ylheng
 @Date: 2022-03-24 17:49:12
 @LastEditTime: 2022-03-29 11:22:06
 @LastEditors: JoeyforJoy
 @Description: Transfer rosbag to images
 @Example: python bag2img.py ${bag_file} ${img_topic}
'''

import rosbag
import os
import argparse

from src.b2x.utils import *

def parse_args():
    parser = argparse.ArgumentParser(description="Transfer rosbag to images.")
    parser.add_argument("bag_file", type=str, help = "the path of bag file")
    parser.add_argument("topic", type=str, help = "topic name")
    parser.add_argument("--output_dir", type=str, default="./data/imgs", help = "directory of output images")
    parser.add_argument("--format", type=str, default="png", help = "format of output images (png, jpg, jpeg...)")
    return parser.parse_args()

if __name__ == "__main__":
    args = parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    bag = rosbag.Bag(args.bag_file, "r")
    bag_data = bag.read_messages(args.topic)
    
    for topic, msg, t in bag_data:
        dumpImageMsg(msg, args.output_dir, str(t), compressed = isCompressedImage(topic), format = args.format)