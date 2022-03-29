'''
 @Author: JoeyforJoy
 @Date: 2022-03-25 19:39:52
 @LastEditTime: 2022-03-29 11:08:00
 @LastEditors: JoeyforJoy
 @Description: Transfer rosbag to synchronized image1 and image2.
 @Example: python bag2sync_img2.py ${bag_file} ${img1_topic} ${img2_topic}
'''

import os
from os import system
import argparse

def parse_args():
    parser = argparse.ArgumentParser(description="Transfer rosbag to images.")
    parser.add_argument("bag_file", type=str, help = "the path of bag file")
    parser.add_argument("topic_img1", type=str, help = "the name of the image1 topic")
    parser.add_argument("topic_img2", type=str, help = "the name of the image2 topic")
    parser.add_argument("--output_dir", type=str, default="./data/synchronized", 
                        help = "the root directory of the output files")
    parser.add_argument("--img1_dir_label", type=str, default="pcd", 
                        help = "the subdirectory name of output pcds")
    parser.add_argument("--img2_dir_label", type=str, default="image", 
                        help = "the subdirectory name of output images")
    parser.add_argument("--tot", type=float, default=0.01, 
                        help = "the tolerence of time synchronization")
    return parser.parse_args()

if __name__ == "__main__":
    args = parse_args()

    if not os.path.exists("./devel/setup.bash"):
        print("ERROR. './devel/setup.bash' must exist")
        exit()
    
    system("roscore&")
    system("./devel/setup.bash; \
            rosrun b2x time_sync_cam2.py %s %s --output_dir %s --tot %s \
                --img1_dir_label %s --img2_dir_label %s &" % 
            (args.topic_img1, args.topic_img2, args.output_dir, args.tot, \
             args.img1_dir_label, args.img2_dir_label))
    system("rosbag play %s" % (args.bag_file))
    system("killall rosbag; killall roscore; sleep 1;")
