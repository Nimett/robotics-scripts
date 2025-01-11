"""
This script converts a ROS1 bag file to a ROS2 bag file. 
Currenly it supports CompressedImage and TFMessage types.

Note:
- Run this script in a ROS2 environment.
- Adjust the topics_to_include dictionary to include the topics you want to convert.
"""

import argparse
from pathlib import Path

import rclpy
from rclpy.serialization import serialize_message

import rosbag2_py
from rosbags.highlevel import AnyReader

from builtin_interfaces.msg import Time
from sensor_msgs.msg import CompressedImage as ROS2CompressedImage
from tf2_msgs.msg import TFMessage as ROS2TFMessage
from geometry_msgs.msg import TransformStamped as ROS2TransformStamped


# Topics from the ROS1 bag file that will be written to the ROS2 bag file
topics_to_include = {
   "/front_cam/stereo/image_rect_color/compressed": "sensor_msgs/msg/CompressedImage",
   "/rear_cam/stereo/image_rect_color/compressed": "sensor_msgs/msg/CompressedImage",
   "/tf": "tf2_msgs/msg/TFMessage",
   "/tf_static": "tf2_msgs/msg/TFMessage",
}

def generate_ros2_compressed_image(ros1_msg):
    ros2_msg = ROS2CompressedImage()
    ros2_msg.header.stamp = Time(sec=ros1_msg.header.stamp.sec, nanosec=ros1_msg.header.stamp.nanosec)
    ros2_msg.header.frame_id = ros1_msg.header.frame_id
    ros2_msg.format = ros1_msg.format
    ros2_msg.data = ros1_msg.data.tolist()
    
    return ros2_msg


def generate_ros2_tf_tree(ros1_msg):
    ros2_msg = ROS2TFMessage()

    for transform in ros1_msg.transforms:
        ros2_transform_stamped = ROS2TransformStamped()
        ros2_transform_stamped.header.stamp = Time(sec=transform.header.stamp.sec, nanosec=transform.header.stamp.nanosec)
        ros2_transform_stamped.header.frame_id = transform.header.frame_id
        ros2_transform_stamped.child_frame_id = transform.child_frame_id
        ros2_transform_stamped.transform.translation.x = transform.transform.translation.x
        ros2_transform_stamped.transform.translation.y = transform.transform.translation.y
        ros2_transform_stamped.transform.translation.z = transform.transform.translation.z
        ros2_transform_stamped.transform.rotation.x = transform.transform.rotation.x
        ros2_transform_stamped.transform.rotation.y = transform.transform.rotation.y
        ros2_transform_stamped.transform.rotation.z = transform.transform.rotation.z
        ros2_transform_stamped.transform.rotation.w = transform.transform.rotation.w

        ros2_msg.transforms.append(ros2_transform_stamped)
    
    return ros2_msg


def convert_ros1_to_ros2(ros1_bag_path, ros2_bag_path):
    rclpy.init()

    writer = rosbag2_py.SequentialWriter()
    storage_options = rosbag2_py.StorageOptions(uri=ros2_bag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("", "")
    writer.open(storage_options, converter_options)

    # Read the ROS1 bag
    with AnyReader([Path(ros1_bag_path)]) as bag:
        
        for connection, timestamp, rawdata in bag.messages():
            if connection.topic in topics_to_include and connection.msgtype == topics_to_include[connection.topic]:
                
                msg = bag.deserialize(rawdata, connection.msgtype)
                
                topic_metadata = rosbag2_py.TopicMetadata(
                    name=connection.topic,
                    type=connection.msgtype,
                    serialization_format="cdr"
                )

                writer.create_topic(topic_metadata)

                if connection.msgtype == "sensor_msgs/msg/CompressedImage":
                    ros2_msg = generate_ros2_compressed_image(msg)
                elif connection.msgtype == "tf2_msgs/msg/TFMessage":
                    ros2_msg = generate_ros2_tf_tree(msg)
                else:
                    raise ValueError(f"Unsupported message type: {connection.msgtype}")

                ros2_msg = serialize_message(ros2_msg)
                writer.write(connection.topic, ros2_msg, timestamp)

    rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert ROS1 bag to ROS2 bag")
    parser.add_argument("--ros1_bag_path", type=str, help="Path to the ROS1 bag file")
    parser.add_argument("--ros2_bag_path", type=str, help="Path to the ROS2 bag file")

    args = parser.parse_args()

    convert_ros1_to_ros2(args.ros1_bag_path, args.ros2_bag_path)

    print("Conversion complete!")