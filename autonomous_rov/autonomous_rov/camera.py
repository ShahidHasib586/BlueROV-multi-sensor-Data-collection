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

class USBCamera(Node):
    def __init__(self):
        super().__init__('usb_camera_node')
        self.publisher = self.create_publisher(Image, 'usb_camera/image_raw', 10)
        self.timer = self.create_timer(0.03, self.timer_callback)
        self.bridge = CvBridge()

        
        # GStreamer pipeline for UDP stream
        self.cap = cv2.VideoCapture('/dev/video4') 

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
        self.cap = cv2.VideoCapture('/dev/video6')

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

def main():
    already_initialized = rclpy.ok()
    if not already_initialized:
        rclpy.init()


    usb_node = USBCamera()
    pc_node = PCCamera()

    executor = MultiThreadedExecutor(num_threads=2)
 
    executor.add_node(usb_node)
    executor.add_node(pc_node)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        while rclpy.ok():


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

        usb_node.cap.release()
        pc_node.cap.release()

        usb_node.destroy_node()
        pc_node.destroy_node()
        cv2.destroyAllWindows()

        if not already_initialized:
            rclpy.shutdown()


if __name__ == '__main__':
    main()
