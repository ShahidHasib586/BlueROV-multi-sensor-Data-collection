import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

def convert_bag_to_video(bag_path, topic_name, output_path, fps=30):
    bridge = CvBridge()

    # Open the bag
    reader = SequentialReader()
    #storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    storage_options = StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    # Get topic types and check for the selected topic
    topic_types = reader.get_all_topics_and_types()
    topic_type_dict = {t.name: t.type for t in topic_types}
    if topic_name not in topic_type_dict:
        raise ValueError(f"Topic {topic_name} not found in bag.")

    # Set filter to only selected topic
    #reader.set_filter({'topics': [topic_name]})
    reader.set_filter(StorageFilter(topics=[topic_name]))


    # Prepare video writer
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_writer = None
    frame_size = None

    print("Starting conversion...")

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic != topic_name:
            continue

        msg = deserialize_message(data, Image)

        try:
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            print("Skipping frame due to error:", e)
            continue

        if frame_size is None:
            frame_size = (cv_image.shape[1], cv_image.shape[0])
            video_writer = cv2.VideoWriter(output_path, fourcc, fps, frame_size)

        video_writer.write(cv_image)

    if video_writer:
        video_writer.release()
        print(f"Video saved to: {output_path}")
    else:
        print("No video written (no valid frames).")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', required=True, help='Path to ROS 2 bag folder')
    parser.add_argument('--topic', required=True, help='Topic name to convert')
    parser.add_argument('--out', required=True, help='Output MP4 file path')
    parser.add_argument('--fps', type=int, default=30, help='Frames per second')
    args = parser.parse_args()

    rclpy.init()
    convert_bag_to_video(args.bag, args.topic, args.out, args.fps)
    rclpy.shutdown()
