#!/usr/bin/env python3
"""ROS2 node that monitors AUV pose against volumetric triggers.

Subscribes to odometry time, checks AABB containment for each trigger zone,
publishes RViz markers for visualization, and exits with success/failure
once all triggers are resolved.
"""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

from sim_test_harness.scn_trigger_parser import parse_triggers


# Trigger states
PENDING = 0
PASSED = 1
FAILED = 2

# Colors (RGBA)
COLOR_PENDING = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.3)   # yellow
COLOR_PASSED = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.3)     # green
COLOR_FAILED = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.3)     # red

# Text colors (opaque)
TEXT_PENDING = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
TEXT_PASSED = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
TEXT_FAILED = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)


class VolumeTriggerNode(Node):
    def __init__(self):
        super().__init__("volume_trigger_node")

        self.declare_parameter("scn_file_path", "")
        scn_path = self.get_parameter("scn_file_path").value
        if not scn_path:
            self.get_logger().fatal("Required parameter 'scn_file_path' not set!")
            raise SystemExit(1)

        self.triggers = parse_triggers(scn_path)
        if not self.triggers:
            self.get_logger().warn("No <volumetric_trigger> elements found in %s" % scn_path)
            raise SystemExit(0)

        self.get_logger().info(
            "Loaded %d volumetric trigger(s) from %s" % (len(self.triggers), scn_path)
        )
        for t in self.triggers:
            self.get_logger().info(
                "  [%s] pos=(%.1f, %.1f, %.1f) dims=(%.1f, %.1f, %.1f) timeout=%.1fs"
                % (t.name, *t.position, *t.dimensions, t.timeout_s)
            )

        self.states = [PENDING] * len(self.triggers)
        self.start_time = time.monotonic()

        # Odometry subscription
        self.odom_sub = self.create_subscription(
            Odometry, "/hydrus/odometry", self._odom_cb, 10
        )

        # RViz marker publisher (transient local so RViz picks it up late)
        marker_qos = QoSProfile(depth=10)
        marker_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.marker_pub = self.create_publisher(
            MarkerArray, "/hydrus/test_triggers", marker_qos
        )

        # 1 Hz timer for deadline checking & marker updates
        self.check_timer = self.create_timer(1.0, self._check_deadlines)

        # Publish initial markers
        self._publish_markers()

    def _is_inside(self, pos, trigger) -> bool:
        """Axis-aligned bounding box containment check."""
        tx, ty, tz = trigger.position
        dx, dy, dz = trigger.dimensions
        return (
            abs(pos[0] - tx) <= dx / 2.0
            and abs(pos[1] - ty) <= dy / 2.0
            and abs(pos[2] - tz) <= dz / 2.0
        )

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        pos = (p.x, p.y, p.z)

        for i, trigger in enumerate(self.triggers):
            if self.states[i] != PENDING:
                continue
            if self._is_inside(pos, trigger):
                self.states[i] = PASSED
                elapsed = time.monotonic() - self.start_time
                self.get_logger().info(
                    "PASSED trigger '%s' at %.1fs (pos=%.2f, %.2f, %.2f)"
                    % (trigger.name, elapsed, *pos)
                )
                self._publish_markers()
                self._check_all_resolved()

    def _check_deadlines(self):
        elapsed = time.monotonic() - self.start_time
        changed = False

        for i, trigger in enumerate(self.triggers):
            if self.states[i] != PENDING:
                continue
            remaining = trigger.timeout_s - elapsed
            if remaining <= 0:
                self.states[i] = FAILED
                self.get_logger().error(
                    "FAILED trigger '%s' — timeout %.1fs exceeded"
                    % (trigger.name, trigger.timeout_s)
                )
                changed = True

        if changed:
            self._publish_markers()
            self._check_all_resolved()
        else:
            # Log status periodically
            pending = [
                t.name for t, s in zip(self.triggers, self.states) if s == PENDING
            ]
            if pending:
                self.get_logger().info(
                    "Waiting (%.1fs): %s" % (elapsed, ", ".join(pending))
                )

    def _check_all_resolved(self):
        if any(s == PENDING for s in self.states):
            return

        passed = sum(1 for s in self.states if s == PASSED)
        failed = sum(1 for s in self.states if s == FAILED)
        total = len(self.states)
        elapsed = time.monotonic() - self.start_time

        self.get_logger().info("=" * 50)
        self.get_logger().info(
            "ALL TRIGGERS RESOLVED in %.1fs: %d/%d passed, %d/%d failed"
            % (elapsed, passed, total, failed, total)
        )
        for t, s in zip(self.triggers, self.states):
            status = "PASSED" if s == PASSED else "FAILED"
            self.get_logger().info("  [%s] %s" % (t.name, status))
        self.get_logger().info("=" * 50)

        # Give time for the log to flush and markers to publish
        self.create_timer(1.0, lambda: self._shutdown(failed > 0))

    def _shutdown(self, has_failures: bool):
        self.get_logger().info(
            "Exiting with code %d" % (1 if has_failures else 0)
        )
        raise SystemExit(1 if has_failures else 0)

    def _publish_markers(self):
        marker_array = MarkerArray()

        for i, trigger in enumerate(self.triggers):
            state = self.states[i]

            # Box marker for the volume
            box = Marker()
            box.header.frame_id = "world_ned"
            box.header.stamp = self.get_clock().now().to_msg()
            box.ns = "volume_triggers"
            box.id = i * 2
            box.type = Marker.CUBE
            box.action = Marker.ADD

            box.pose.position.x = trigger.position[0]
            box.pose.position.y = trigger.position[1]
            box.pose.position.z = trigger.position[2]
            box.pose.orientation.w = 1.0

            box.scale.x = trigger.dimensions[0]
            box.scale.y = trigger.dimensions[1]
            box.scale.z = trigger.dimensions[2]

            if state == PENDING:
                box.color = COLOR_PENDING
            elif state == PASSED:
                box.color = COLOR_PASSED
            else:
                box.color = COLOR_FAILED

            marker_array.markers.append(box)

            # Text label above the volume
            text = Marker()
            text.header.frame_id = "world_ned"
            text.header.stamp = self.get_clock().now().to_msg()
            text.ns = "volume_trigger_labels"
            text.id = i * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD

            text.pose.position.x = trigger.position[0]
            text.pose.position.y = trigger.position[1]
            text.pose.position.z = trigger.position[2] - 2.0  # above in NED (z-down)

            elapsed = time.monotonic() - self.start_time
            if state == PENDING:
                remaining = max(0.0, trigger.timeout_s - elapsed)
                label = "%s (%.0fs left)" % (trigger.name, remaining)
                text.color = TEXT_PENDING
            elif state == PASSED:
                label = "%s PASSED" % trigger.name
                text.color = TEXT_PASSED
            else:
                label = "%s FAILED" % trigger.name
                text.color = TEXT_FAILED

            text.text = label
            text.scale.z = 0.4  # text height

            marker_array.markers.append(text)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = VolumeTriggerNode()
        rclpy.spin(node)
    except SystemExit as e:
        rclpy.try_shutdown()
        sys.exit(e.code)
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
