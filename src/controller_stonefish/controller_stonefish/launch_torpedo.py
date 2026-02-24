#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from std_msgs.msg import Float64
import time


class LaunchTorpedoNode(Node):
    def __init__(self):
        super().__init__('launch_torpedo')

        # Service client to release the glue
        self.attach_client = self.create_client(SetBool, '/torpedo/attach')

        # Publisher to apply push force
        self.push_pub = self.create_publisher(Float64, '/torpedo/push', 10)

        self.get_logger().info('Launch Torpedo Node initialized.')


def main(args=None):
    rclpy.init(args=args)
    node = LaunchTorpedoNode()

    # Wait for the attach service to be available
    while not node.attach_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for /torpedo/attach service...')

    # 1. Release the glue
    node.get_logger().info('Releasing torpedo glue...')
    req = SetBool.Request()
    req.data = False
    future = node.attach_client.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(
            f'Glue release response: {future.result().message}')
    else:
        node.get_logger().error('Failed to call /torpedo/attach service')
        node.destroy_node()
        rclpy.shutdown()
        return

    # 2. Apply push force
    node.get_logger().info('Applying push force...')
    push_msg = Float64()
    push_msg.data = 0.2  # Max force according to specs

    # Publish force for a short duration
    start_time = time.time()
    while time.time() - start_time < 0.5:
        node.push_pub.publish(push_msg)
        rclpy.spin_once(node, timeout_sec=0.1)

    # 3. Stop push force
    node.get_logger().info('Stopping push force...')
    push_msg.data = 0.0
    node.push_pub.publish(push_msg)

    node.get_logger().info('Torpedo launched successfully.')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
