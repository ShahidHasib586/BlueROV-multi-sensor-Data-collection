#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time


class BlueROVCamera(Node):
    def __init__(self):
        super().__init__('bluerov_camera_node')
        self.publisher = self.create_publisher(Image, 'bluerov_camera/image_raw', 10)
        self.timer = self.create_timer(0.03, self.timer_callback)  # ~30 FPS
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture("udpsrc port=5600 ! application/x-rtp, encoding-name=H264 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink", cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open BlueROV camera stream.")
        else:
            self.get_logger().info("BlueROV camera stream opened")
        self.latest_frame = None

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(msg)
            self.latest_frame = frame
        else:
            self.get_logger().warn("No frame received from BlueROV camera")


class USBCamera(Node):
    def __init__(self):
        super().__init__('usb_camera_node')
        self.publisher = self.create_publisher(Image, 'usb_camera/image_raw', 10)
        self.timer = self.create_timer(0.03, self.timer_callback)  # ~30 FPS
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)  # Change index if needed
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open USB camera.")
        else:
            self.get_logger().info("USB camera opened")
        self.latest_frame = None

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(msg)
            self.latest_frame = frame
        else:
            self.get_logger().warn("No frame received from USB camera")


def main(args=None):
    rclpy.init(args=args)

    bluerov_node = BlueROVCamera()
    usb_node = USBCamera()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(bluerov_node)
    executor.add_node(usb_node)

    try:
        while rclpy.ok():
            # Process ROS messages
            #rclpy.spin_once(executor, timeout_sec=0.01)
            executor.spin_once(timeout_sec=0.01)

            # Display frames in the main thread
            if bluerov_node.latest_frame is not None:
                cv2.imshow("BlueROV Camera", bluerov_node.latest_frame)

            if usb_node.latest_frame is not None:
                cv2.imshow("USB Camera", usb_node.latest_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            time.sleep(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        bluerov_node.cap.release()
        usb_node.cap.release()
        bluerov_node.destroy_node()
        usb_node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

