#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor

class BlueROVCamera(Node):
    def __init__(self):
        super().__init__('bluerov_camera_node')
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, 'bluerov/image', 10)

        # GStreamer pipeline for UDP stream
        self.cap = cv2.VideoCapture(
            "udpsrc port=5600 ! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink",
            cv2.CAP_GSTREAMER
        )

        if not self.cap.isOpened():
            self.get_logger().error("Could not open BlueROV camera stream")
        else:
            self.get_logger().info("BlueROV camera stream opened")

        self.timer = self.create_timer(1.0 / 30, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            cv2.imshow("BlueROV Camera", frame)
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(msg)
            self.get_logger().info("Frame received")
        else:
            self.get_logger().warn("No frame from BlueROV camera")

        cv2.waitKey(1)


class USBCamera(Node):
    def __init__(self):
        super().__init__('usb_camera_node')
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, 'usb_cam/image', 10)

        # GStreamer pipeline for UDP stream
        self.cap = cv2.VideoCapture(
            "udpsrc port=5602 ! application/x-rtp, encoding-name=JPEG,payload=26 ! rtpjpegdepay ! jpegdec ! videoconvert ! appsink",
            cv2.CAP_GSTREAMER
        )

        if not self.cap.isOpened():
            self.get_logger().error("Could not open USB camera at /dev/video4")
        else:
            self.get_logger().info("USB camera opened")

        self.timer = self.create_timer(1.0 / 30, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            cv2.imshow("USB Camera", frame)
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(msg)
            self.get_logger().info("Frame received")
        else:
            self.get_logger().warn("No frame from USB camera")

        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    bluerov_node = BlueROVCamera()
    usb_node = USBCamera()

    executor = MultiThreadedExecutor(num_threads=1)
    executor.add_node(bluerov_node)
    executor.add_node(usb_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        bluerov_node.destroy_node()
        usb_node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
