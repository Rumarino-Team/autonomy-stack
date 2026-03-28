import rclpy

import math
import threading
import time

from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Pose, PoseWithCovariance, Twist, TwistWithCovariance


class ImuPoseBased(Node):
    def __init__(self):
        super().__init__("IMUPoseBased")
        
        self.declare_parameter('odometry_topic', '/hydrus/odometry')
        self.declare_parameter('imu_topic', '/hydrus/imu')
        self.declare_parameter('merged_odometry_topic', '/hydrus/merged_odometry')
        self.declare_parameter('control_period', 0.1)
        
        self.odometry_topic = str(self.get_parameter('odometry_topic').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.merged_odometry_topic = str(self.get_parameter('merged_odometry_topic').value)
        self.control_period = float(self.get_parameter('control_period').value)
        
        self.publisher_ = self.create_publisher(Odometry, self.merged_odometry_topic, 10)
        
        self.create_subscription(
            Odometry, self.odometry_topic, self.odometry_callback, 10
        )
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)

        self.timer = self.create_timer(self.control_period, self.timer_callback)
        
        self.state_lock = threading.Lock()
        self.current_position = None
        self.current_quaternion = None
        self.last_odom_time = None
        self.last_no_imu_warn = 0.0
        
        self.get_logger().info(f'Using odometry topic: {self.odometry_topic}')
        self.get_logger().info(f'Using IMU topic: {self.imu_topic}')
        self.get_logger().info(f'Publishing merged odometry on: {self.merged_odometry_topic}')
        self.get_logger().info('Waiting for odometry and IMU data...')


    def odometry_callback(self, msg):
        p = msg.pose.pose.position
        with self.state_lock:
            self.current_position = [p.x, p.y, p.z]
            self.last_odom_time = time.monotonic()
        self.get_logger().debug(f'Received odometry: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}')

    def imu_callback(self, msg):
        o = msg.orientation
        quat_norm = math.sqrt(o.w * o.w + o.x * o.x + o.y * o.y + o.z * o.z)
        if quat_norm < 1e-9:
            now = time.monotonic()
            with self.state_lock:
                if now - self.last_no_imu_warn >= 1.0:
                    self.get_logger().warning('Received invalid IMU quaternion with near-zero norm.')
                    self.last_no_imu_warn = now
            return

        quaternion_wxyz = (
            o.w / quat_norm,
            o.x / quat_norm,
            o.y / quat_norm,
            o.z / quat_norm,
        )

        with self.state_lock:
            self.current_quaternion = quaternion_wxyz
        self.get_logger().debug(f'Received IMU: w={quaternion_wxyz[0]:.3f}, x={quaternion_wxyz[1]:.3f}, y={quaternion_wxyz[2]:.3f}, z={quaternion_wxyz[3]:.3f}')

    def timer_callback(self):
        """Merge odometry position with IMU rotation and publish."""
        with self.state_lock:
            current_position = self.current_position
            current_quaternion = self.current_quaternion
        
        if current_position is None or current_quaternion is None:
            if current_position is None:
                self.get_logger().debug('Waiting for odometry...')
            if current_quaternion is None:
                self.get_logger().debug('Waiting for IMU data...')
            return
        
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        odom_msg.pose.pose.position.x = current_position[0]
        odom_msg.pose.pose.position.y = current_position[1]
        odom_msg.pose.pose.position.z = current_position[2]
        
        odom_msg.pose.pose.orientation.w = current_quaternion[0]
        odom_msg.pose.pose.orientation.x = current_quaternion[1]
        odom_msg.pose.pose.orientation.y = current_quaternion[2]
        odom_msg.pose.pose.orientation.z = current_quaternion[3]
        
        self.publisher_.publish(odom_msg)
        self.get_logger().debug(f'Published merged odometry: pos=({current_position[0]:.3f}, {current_position[1]:.3f}, {current_position[2]:.3f})')


def main(args=None):
    rclpy.init(args=args)
    imu_pose_based = ImuPoseBased()
    rclpy.spin(imu_pose_based)
    imu_pose_based.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

