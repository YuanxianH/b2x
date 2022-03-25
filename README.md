# b2x
![license](https://img.shields.io/apm/l/vim-mode?style=plastic) ![license](https://img.shields.io/badge/platform-linux-blue?style=plastic)

**b2x** (bag to everything) can transfer rosbag to images, pcd files and so on.

Now, we surpport:
- rosbag to images
- rosbag to pcd files
- get synchronized images and pcd files from a rosbag

## Environment
This project can only run in **linux** now.

## Dependency
- cmake
- ros
- opencv-python

## QuickStart
### Build before run
```bash
catkin_make
```

### Run
You can run the scripts, named `bag2***.py` in the root of this project, to transfer bag files.

In the below, `${}` means you should replace the name in your cases.

The result will be stored in `data/` directory in default. Also, you can specify the output directory by `--output_dir` option.

You can find other options of the script you want execute by add `--help` option.

Now, we surpport:
- **rosbag to images**: 
```bash
# output in the default directory
python bag2img.py ${bag_path} ${image_topic}
# output in specified directory
python bag2img.py ${bag_path} ${image_topic} --output_dir ${output_dir} 
```

- **rosbag to images**: 
```bash
python bag2pcd.py ${bag_path} ${pcd_topic}
```

- **get synchronized images and pcd files from a rosbag**:
```bash
python bag2sync_img_pcd.py ${bag_path} ${image_topic} ${pcd_topic}
```

