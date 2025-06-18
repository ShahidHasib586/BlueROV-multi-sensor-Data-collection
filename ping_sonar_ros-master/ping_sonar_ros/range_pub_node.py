import rclpy
from rclpy.executors import MultiThreadedExecutor
from .range_pub_component import RangePublisher

def main(args=None):
    rclpy.init(args=args)
    range_publisher = RangePublisher()

    executor = MultiThreadedExecutor(num_threads=4)  # adjust number of threads as needed
    executor.add_node(range_publisher)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        range_publisher.destroy_node()
        rclpy.shutdown()
