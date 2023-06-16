# Make sure to have CoppeliaSim running, with followig scene loaded:
#
# scenes/messaging/.ttt
#
# Do not launch simulation, then run this script
from utilities.simulation import Simulation as simx
import sys, os
from threading import Thread
from rclpy.executors import MultiThreadedExecutor
import rclpy

print('Program started')

def main(args=None):
    rclpy.init(args=args)

    simulator_node = simx()
    executor = MultiThreadedExecutor()
    executor.add_node(simulator_node)

    # Start the ROS2 node on a separate thread
    thread = Thread(target=executor.spin)
    thread.start()
    simulator_node.get_logger().info("Spinned ROS2 Node . . .")

    # Let the app running on the main thread
    try:
        sys.exit(simulator_node.exec())

    finally:
        simulator_node.get_logger().info("Shutting down ROS2 Node . . .")
        simulator_node.destroy_node()
        executor.shutdown()


if __name__ == '__main__':
    main()
# Wait until above movement sequence finished executing:
# simulator.waitForMovementExecuted('up')