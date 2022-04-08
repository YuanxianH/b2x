'''
 @Author: JoeyforJoy
 @Date: 2022-03-25 19:39:52
 @LastEditTime: 2022-04-08 11:56:45
 @LastEditors: JoeyforJoy
 @Description: Transfer rosbag to synchronized image and pcd files.
 @Example: python bag2sync_img_pts.py ${bag_file} ${img_topic} ${pcd_topic}
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
    parser.add_argument("--pts_fmt", type=str, default="pcd",
                        help = "format of point cloud file, eg. pcd, bin(kitti)")
    return parser.parse_args()

if __name__ == "__main__":
    args = parse_args()

    curdir = os.path.abspath(os.path.dirname(__file__)) # current directory
    setup_path = os.path.join(curdir, "devel", "setup.sh")

    if not os.path.exists(setup_path):
        print("ERROR. './devel/setup.bash' must exist")
        exit()
    
    system("roscore &")
    # NOTE: os.system() can only source sh instead of bash
    system(". %s; \
            rosrun b2x time_sync_lidar_cam.py %s %s --output_dir %s --tot %s \
                --pcd_dir_label %s --img_dir_label %s --pts_fmt %s&" % 
            (setup_path, args.topic_img, args.topic_lidar, args.output_dir, args.tot, \
             args.pcd_dir_label, args.img_dir_label, args.pts_fmt))
    system("rosbag play %s" % (args.bag_file))
    system("killall rosbag; killall roscore; killall time_sync_lidar_cam.py; sleep 1;")
    