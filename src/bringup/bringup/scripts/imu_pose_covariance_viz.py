import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu


class ImuPoseCovarianceViz(Node):
    def __init__(self) -> None:
        super().__init__("imu_pose_covariance_viz")

        self.declare_parameter("imu_topic", "/bridge/imu")
        self.declare_parameter("output_topic", "/imu/pose_with_covariance")
        self.declare_parameter("default_frame_id", "map")
        self.declare_parameter("position_variance", 1e6)
        self.declare_parameter("fallback_orientation_variance", 0.1)

        imu_topic = self.get_parameter("imu_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value

        self._default_frame_id = (
            self.get_parameter("default_frame_id").get_parameter_value().string_value
        )
        self._position_variance = (
            self.get_parameter("position_variance").get_parameter_value().double_value
        )
        self._fallback_orientation_variance = (
            self.get_parameter("fallback_orientation_variance")
            .get_parameter_value()
            .double_value
        )

        self._pub = self.create_publisher(PoseWithCovarianceStamped, output_topic, 10)
        self._sub = self.create_subscription(Imu, imu_topic, self._imu_cb, 10)

        self.get_logger().info(
            f"Relaying IMU '{imu_topic}' to PoseWithCovarianceStamped '{output_topic}'"
        )

    def _imu_cb(self, msg: Imu) -> None:
        out = PoseWithCovarianceStamped()

        out.header.stamp = msg.header.stamp
        out.header.frame_id =  self._default_frame_id

        out.pose.pose.position.x = 0.0
        out.pose.pose.position.y = 0.0
        out.pose.pose.position.z = 0.0
        out.pose.pose.orientation = msg.orientation

        cov = [0.0] * 36

        # Position is unknown from IMU alone, so keep high uncertainty on xyz.
        cov[0] = self._position_variance
        cov[7] = self._position_variance
        cov[14] = self._position_variance

        # IMU orientation covariance is 3x3 for roll/pitch/yaw in row-major order.
        if msg.orientation_covariance[0] >= 0.0:
            for r in range(3):
                for c in range(3):
                    cov[(r + 3) * 6 + (c + 3)] = msg.orientation_covariance[r * 3 + c]
        else:
            cov[21] = self._fallback_orientation_variance
            cov[28] = self._fallback_orientation_variance
            cov[35] = self._fallback_orientation_variance
        
        out.pose.covariance = cov
        # self._logger.info(out._pose.x)
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuPoseCovarianceViz()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
