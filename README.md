# Robotics Scripts

## Generate ROS2 Bag files from ROS1

**Location:** `ros2/generate_ros2_bag.py`

This script converts a ROS1 bag file to a ROS2 bag file. 
Currenly it supports CompressedImage and TFMessage types.

- Run this script in a ROS2 environment.
- Adjust the topics_to_include dictionary to include the topics you want to convert.


## ROS2 NVIDIA Docker 
**Location:** `ros2_docker`

- Install Docker extension on VSCode
- Open in Dev Container

```bash
cd /home/root/humble_ws
source /home/root/.bashrc
ros2 run simple_pub simple pub
```