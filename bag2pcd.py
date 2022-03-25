'''
 @Author: JoeyforJoy & ylheng
 @Date: 2022-03-24 18:16:03
 @LastEditTime: 2022-03-25 22:04:02
 @LastEditors: JoeyforJoy
 @Description: rosbag 2 pcd
 @Example: python bag2pcd.py ${bag_file} ${pcd_topic}
'''

import os
from os import system
import argparse

def parse_args():
    parser = argparse.ArgumentParser(description="Transfer rosbag to images.")
    parser.add_argument("bag_file", type=str, help = "the path of bag file")
    parser.add_argument("topic", type=str, help = "topic name")
    parser.add_argument("--output_dir", type=str, default="./data/pcd", help = "directory of output pcd files")
    return parser.parse_args()

if __name__ == "__main__":
    args = parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    system("roscore&")
    system("rosrun pcl_ros bag_to_pcd %s %s %s; " % (args.bag_file, args.topic, args.output_dir))
    system("killall roscore; sleep 1;")
