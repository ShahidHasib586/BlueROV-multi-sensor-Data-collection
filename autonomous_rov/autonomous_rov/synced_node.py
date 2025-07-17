import rclpy
from rclpy.node import Node
from threading import Thread

from autonomous_rov.multivideo import main as run_camera_main
#from ping360_sonar.src.ping360 import run_sonar_main
from ping360_sonar.ping360 import run_sonar_main


class SyncedNode(Node):
    def __init__(self):
        super().__init__('synced_node')
        self.get_logger().info("Launching synchronized camera and sonar threads")

        self.camera_thread = Thread(target=self.run_cameras, daemon=True)
        self.sonar_thread = Thread(target=self.run_sonar, daemon=True)

        self.camera_thread.start()
        self.sonar_thread.start()

    def run_cameras(self):
        self.get_logger().info("Starting camera processing...")
        run_camera_main()

    def run_sonar(self):
        self.get_logger().info("Starting sonar processing...")
        run_sonar_main()

def main(args=None):
    rclpy.init(args=args)
    node = SyncedNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
