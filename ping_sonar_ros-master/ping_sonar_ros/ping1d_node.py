import rclpy
from rclpy.executors import MultiThreadedExecutor
from .ping1d_component import Ping1dComponent

def main(args=None):
    rclpy.init(args=args)
    range_publisher = Ping1dComponent()

    executor = MultiThreadedExecutor(num_threads=4)  # or choose the number of threads you need
    executor.add_node(range_publisher)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        range_publisher.destroy_node()
        rclpy.shutdown()
