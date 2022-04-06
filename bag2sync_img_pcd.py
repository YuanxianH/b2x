'''
 @Author: JoeyforJoy
 @Date: 2022-03-25 19:39:52
 @LastEditTime: 2022-04-06 21:33:24
 @LastEditors: JoeyforJoy
 @Description: Transfer rosbag to synchronized image and pcd files.
 @Example: python bag2sync_img_pcd.py ${bag_file} ${img_topic} ${pcd_topic}
'''

import os
from os import system
import argparse

def parse_args():
    parser = argparse.ArgumentParser(description="Transfer rosbag to images.")
    parser.add_argument("bag_file", type=str, help = "the path of bag file")
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

if __name__ == "__main__":
    args = parse_args()

    if not os.path.exists("./devel/setup.bash"):
        print("ERROR. './devel/setup.bash' must exist")
        exit()
    
    system("roscore &")
    # NOTE: os.system() can only source sh instead of bash
    system(". ./devel/setup.sh; \
            rosrun b2x time_sync_lidar_cam.py %s %s --output_dir %s --tot %s \
                --pcd_dir_label %s --img_dir_label %s&  " % 
            (args.topic_img, args.topic_lidar, args.output_dir, args.tot, \
             args.pcd_dir_label, args.img_dir_label))
    system("rosbag play %s" % (args.bag_file))
    system("killall rosbag; killall roscore; killall time_sync_lidar_cam.py; sleep 1;")
    