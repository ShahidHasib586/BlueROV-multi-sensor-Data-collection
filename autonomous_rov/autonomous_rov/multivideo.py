#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import time

# Custom wrapper to always get the latest frame
class BufferlessCapture:
    def __init__(self, src, backend=cv2.CAP_GSTREAMER):
        self.cap = cv2.VideoCapture(src, backend)
        self.frame = None
        self.lock = threading.Lock()
        self.running = True
        t = threading.Thread(target=self._reader, daemon=True)
        t.start()

    def _reader(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.frame = frame

    def read(self):
        with self.lock:
            return self.frame

    def release(self):
        self.running = False
        self.cap.release()


class BlueROVCamera(Node):
    def __init__(self):
        super().__init__('bluerov_camera_node')
        self.publisher = self.create_publisher(Image, 'bluerov_camera/image_raw', 10)
        self.timer = self.create_timer(0.03, self.timer_callback)
        self.bridge = CvBridge()

        #  Use BufferlessCapture for smoother stream
        gstreamer_pipeline = (
            "udpsrc port=5600 ! application/x-rtp, encoding-name=H264 ! "
            "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink"
        )

        self.cap = BufferlessCapture(gstreamer_pipeline)
        self.latest_frame = None
        self.get_logger().info("BlueROV camera stream opened")

    def timer_callback(self):
        thread_id = threading.get_ident()
        self.get_logger().info(f"Running on thread ID: {thread_id}")
        frame = self.cap.read()
        #time.sleep(1.0)  # Simulate processing delay
        if frame is not None:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(msg)
            self.latest_frame = frame
        else:
            self.get_logger().warn("No frame received from BlueROV camera")


class USBCamera(Node):
    def __init__(self):
        super().__init__('usb_camera_node')
        self.publisher = self.create_publisher(Image, 'usb_camera/image_raw', 10)
        self.timer = self.create_timer(0.03, self.timer_callback)
        self.bridge = CvBridge()

        
        # GStreamer pipeline for UDP stream
        self.cap = cv2.VideoCapture(
            "udpsrc port=5602 ! application/x-rtp, encoding-name=JPEG,payload=26 ! rtpjpegdepay ! jpegdec ! videoconvert ! appsink",
            cv2.CAP_GSTREAMER
        )
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open USB camera.")
        else:
            self.get_logger().info("USB camera opened")

        self.latest_frame = None

    def timer_callback(self):
        thread_id = threading.get_ident()
        self.get_logger().info(f"Running on thread ID: {thread_id}")
        ret, frame = self.cap.read()
        #time.sleep(1.0)  # Simulate processing delay
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(msg)
            self.latest_frame = frame
        else:
            self.get_logger().warn("No frame received from USB camera")

class PCCamera(Node):
    def __init__(self):
        super().__init__('pc_camera_node')
        self.publisher = self.create_publisher(Image, 'pc_camera/image_raw', 10)
        self.timer = self.create_timer(0.03, self.timer_callback)
        self.bridge = CvBridge()

        
        # GStreamer pipeline for UDP stream
        self.cap = cv2.VideoCapture('/dev/video4')

        if not self.cap.isOpened():
            self.get_logger().error("Failed to open PC camera.")
        else:
            self.get_logger().info("PC camera opened")

        self.latest_frame = None

    def timer_callback(self):
        thread_id = threading.get_ident()
        self.get_logger().info(f"Running on thread ID: {thread_id}")
        ret, frame = self.cap.read()
        #time.sleep(1.0)  # Simulate processing delay
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(msg)
            self.latest_frame = frame
        else:
            self.get_logger().warn("No frame received from PC camera")


def main(args=None):
    rclpy.init(args=args)

    bluerov_node = BlueROVCamera()
    usb_node = USBCamera()
    pc_node = PCCamera()

    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(bluerov_node)
    executor.add_node(usb_node)
    executor.add_node(pc_node)

    # Start executor in a separate thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        while rclpy.ok():
            if bluerov_node.latest_frame is not None:
                cv2.imshow("BlueROV Camera", bluerov_node.latest_frame)

            if usb_node.latest_frame is not None:
                cv2.imshow("USB Camera", usb_node.latest_frame)

            if pc_node.latest_frame is not None:
                cv2.imshow("PC Camera", pc_node.latest_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            time.sleep(0.005)

    except KeyboardInterrupt:
        pass
    finally:
        bluerov_node.cap.release()
        usb_node.cap.release()
        pc_node.cap.release()
        bluerov_node.destroy_node()
        usb_node.destroy_node()
        pc_node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()



if __name__ == '__main__':
    main()
