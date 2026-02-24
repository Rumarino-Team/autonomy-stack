#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class DropSphereNode(Node):
    def __init__(self):
        super().__init__('drop_sphere')

        # Service client to release the glue
        self.drop_client = self.create_client(SetBool, '/dropper/drop')

        self.get_logger().info('Drop Sphere Node initialized.')


def main(args=None):
    rclpy.init(args=args)
    node = DropSphereNode()

    # Wait for the drop service to be available
    while not node.drop_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for /dropper/drop service...')

    # 1. Release the glue
    node.get_logger().info('Releasing dropper glue...')
    req = SetBool.Request()
    req.data = False
    future = node.drop_client.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(
            f'Dropper glue release response: {future.result().message}')
    else:
        node.get_logger().error('Failed to call /dropper/drop service')
        node.destroy_node()
        rclpy.shutdown()
        return

    node.get_logger().info('Sphere dropped successfully.')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
