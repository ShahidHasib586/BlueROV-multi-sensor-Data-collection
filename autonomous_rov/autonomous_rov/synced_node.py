import rclpy
from rclpy.node import Node
from threading import Thread

# Import your camera node (assumes it's class-based with a run() method)
from autonomous_rov.multivideo import MultiVideoNode

# Import sonar runner (you must wrap your ping360 main logic in this)
from ping360_sonar.src.ping360 import run_sonar_main  # <-- You must define this!

class SyncedNode(Node):
    def __init__(self):
        super().__init__('synced_node')
        self.get_logger().info("Launching synchronized camera and sonar threads")

        # Launch both camera and sonar in separate threads
        self.camera_thread = Thread(target=self.run_cameras, daemon=True)
        self.sonar_thread = Thread(target=self.run_sonar, daemon=True)

        self.camera_thread.start()
        self.sonar_thread.start()

    def run_cameras(self):
        self.get_logger().info("Starting camera processing...")
        cam_node = MultiVideoNode()
        cam_node.run()  # This should be your main camera logic loop

    def run_sonar(self):
        self.get_logger().info("Starting sonar processing...")
        run_sonar_main()  # This should start the sonar loop

def main(args=None):
    rclpy.init(args=args)
    node = SyncedNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
