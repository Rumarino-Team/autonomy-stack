import threading
import sys
import termios
import time
import tty

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

from controller_stonefish.control_math import (
    KD_DEFAULT,
    KI_DEFAULT,
    KP_DEFAULT,
    THRUSTOR_SATURATE_DEFAULT,
    compute_thruster_command,
    normalize_angle,
    quaternion_to_euler,
)


class ThrusterTeleOp(Node):

    def __init__(self):
        super().__init__('thruster_teleop')

        self.declare_parameter('step_x', 0.1)
        self.declare_parameter('step_y', 0.1)
        self.declare_parameter('step_z', 0.1)
        self.declare_parameter('step_yaw', 0.05)
        self.declare_parameter('control_period', 0.1)
        self.declare_parameter('kp', KP_DEFAULT)
        self.declare_parameter('ki', KI_DEFAULT)
        self.declare_parameter('kd', KD_DEFAULT)
        self.declare_parameter('thruster_saturate', THRUSTOR_SATURATE_DEFAULT)
        self.declare_parameter('enable_joy', True)
        self.declare_parameter('joy_deadzone', 0.15)
        self.declare_parameter('joy_axis_scale', 1.0)
        self.declare_parameter('joy_axis_forward', 1)
        self.declare_parameter('joy_axis_lateral', 0)
        self.declare_parameter('joy_axis_yaw', 2)
        self.declare_parameter('joy_button_up', 4)
        self.declare_parameter('joy_button_down', 5)
        self.declare_parameter('joy_button_hold', 0)
        self.declare_parameter('joy_button_quit', 1)

        self.step_x = float(self.get_parameter('step_x').value)
        self.step_y = float(self.get_parameter('step_y').value)
        self.step_z = float(self.get_parameter('step_z').value)
        self.step_yaw = float(self.get_parameter('step_yaw').value)
        self.control_period = float(self.get_parameter('control_period').value)
        self.kp = [float(v) for v in self.get_parameter('kp').value]
        self.ki = [float(v) for v in self.get_parameter('ki').value]
        self.kd = [float(v) for v in self.get_parameter('kd').value]
        self.thruster_saturate = float(self.get_parameter('thruster_saturate').value)
        self.enable_joy = bool(self.get_parameter('enable_joy').value)
        self.joy_deadzone = float(self.get_parameter('joy_deadzone').value)
        self.joy_axis_scale = float(self.get_parameter('joy_axis_scale').value)
        self.joy_axis_forward = int(self.get_parameter('joy_axis_forward').value)
        self.joy_axis_lateral = int(self.get_parameter('joy_axis_lateral').value)
        self.joy_axis_yaw = int(self.get_parameter('joy_axis_yaw').value)
        self.joy_button_up = int(self.get_parameter('joy_button_up').value)
        self.joy_button_down = int(self.get_parameter('joy_button_down').value)
        self.joy_button_hold = int(self.get_parameter('joy_button_hold').value)
        self.joy_button_quit = int(self.get_parameter('joy_button_quit').value)

        if any(len(v) != 6 for v in (self.kp, self.ki, self.kd)):
            raise ValueError('kp, ki, kd parameters must have exactly 6 values')

        self.publisher_ = self.create_publisher(
            Float64MultiArray, '/hydrus/thrusters', 10
        )
        self.create_subscription(
            Odometry, '/hydrus/odometry', self.odometry_callback, 10
        )
        if self.enable_joy:
            self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.timer = self.create_timer(self.control_period, self.timer_callback)

        self.state_lock = threading.Lock()
        self.running = True

        self.thruster_speeds = [0.0] * 8
        self.current_pose = None
        self.current_quaternion = None
        self.goal_pose = None

        self.sum_err = [0.0] * 6
        self.prev_pose_err = [0.0] * 6
        self.prev_time = None
        self.last_no_odom_warn = 0.0
        self.joy_axes = []
        self.joy_buttons = []
        self.prev_joy_buttons = []

        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

        self.print_instructions()

    def print_instructions(self):
        print("\n=== Hydrus Setpoint Teleop (Mission Executor Controller) ===")
        print('Controls:')
        print(f'  w : Increase x goal by {self.step_x:.2f} m')
        print(f'  s : Decrease x goal by {self.step_x:.2f} m')
        print(f'  r : Increase y goal by {self.step_y:.2f} m')
        print(f'  f : Decrease y goal by {self.step_y:.2f} m')
        print(f'  q : Increase z goal by {self.step_z:.2f} m')
        print(f'  e : Decrease z goal by {self.step_z:.2f} m')
        print(f'  a : Increase yaw goal by {self.step_yaw:.2f} rad')
        print(f'  d : Decrease yaw goal by {self.step_yaw:.2f} rad')
        print('  x : Hold current pose and clear PID history')
        print('  z : Quit')
        if self.enable_joy:
            print('Controller (/joy) mapping (PS5 defaults):')
            print('  Left stick Y : x goal')
            print('  Left stick X : y goal')
            print('  Right stick X: yaw goal')
            print('  L1 / R1      : z up / z down')
            print('  Cross        : hold current pose')
            print('  Circle       : quit')
        print('============================================================\n')

    def keyboard_listener(self):
        """Listen for keyboard input in a separate thread."""
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            while self.running:
                key = sys.stdin.read(1)
                if key:
                    self.process_key(key)
        except Exception as exc:
            self.get_logger().error(f'Keyboard listener error: {exc}')
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def odometry_callback(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        roll, pitch, yaw = quaternion_to_euler(o.w, o.x, o.y, o.z)

        now = time.monotonic()
        current_pose = [p.x, p.y, p.z, roll, pitch, yaw]
        current_quaternion = (o.w, o.x, o.y, o.z)

        with self.state_lock:
            self.current_pose = current_pose
            self.current_quaternion = current_quaternion
            if self.goal_pose is None:
                self.goal_pose = list(current_pose)
                self.sum_err = [0.0] * 6
                self.prev_pose_err = [0.0] * 6
                self.prev_time = now
                self.get_logger().info('Odometry initialized. Goal seeded to current pose.')

    def joy_callback(self, msg):
        with self.state_lock:
            self.joy_axes = list(msg.axes)
            self.joy_buttons = list(msg.buttons)

    def process_key(self, key):
        """Process keyboard input as setpoint updates."""
        key_lower = key.lower()

        if key_lower == 'z':
            self.get_logger().info('Quitting...')
            self.running = False
            self.publish_zero_thrusters()
            rclpy.shutdown()
            return

        with self.state_lock:
            if self.goal_pose is None or self.current_pose is None:
                self.get_logger().warning('Ignoring key input until first odometry is received.')
                return

            if key_lower == 'w':
                self.goal_pose[0] += self.step_x
                self.get_logger().info(f'x goal -> {self.goal_pose[0]:.2f} m')
            elif key_lower == 's':
                self.goal_pose[0] -= self.step_x
                self.get_logger().info(f'x goal -> {self.goal_pose[0]:.2f} m')
            elif key_lower == 'r':
                self.goal_pose[1] += self.step_y
                self.get_logger().info(f'y goal -> {self.goal_pose[1]:.2f} m')
            elif key_lower == 'f':
                self.goal_pose[1] -= self.step_y
                self.get_logger().info(f'y goal -> {self.goal_pose[1]:.2f} m')
            elif key_lower == 'q':
                self.goal_pose[2] += self.step_z
                self.get_logger().info(f'z goal -> {self.goal_pose[2]:.2f} m')
            elif key_lower == 'e':
                self.goal_pose[2] -= self.step_z
                self.get_logger().info(f'z goal -> {self.goal_pose[2]:.2f} m')
            elif key_lower == 'a':
                self.goal_pose[5] = normalize_angle(self.goal_pose[5] + self.step_yaw)
                self.get_logger().info(f'yaw goal -> {self.goal_pose[5]:.2f} rad')
            elif key_lower == 'd':
                self.goal_pose[5] = normalize_angle(self.goal_pose[5] - self.step_yaw)
                self.get_logger().info(f'yaw goal -> {self.goal_pose[5]:.2f} rad')
            elif key_lower == 'x':
                self.goal_pose = list(self.current_pose)
                self.sum_err = [0.0] * 6
                self.prev_pose_err = [0.0] * 6
                self.prev_time = time.monotonic()
                self.get_logger().info('Holding current pose. PID history reset.')

    def _axis_value(self, axes, axis_index, invert=False):
        if axis_index < 0 or axis_index >= len(axes):
            return 0.0
        value = float(axes[axis_index])
        if invert:
            value = -value
        if abs(value) < self.joy_deadzone:
            return 0.0
        return value

    def _button_value(self, buttons, button_index):
        if button_index < 0 or button_index >= len(buttons):
            return 0
        return int(buttons[button_index])

    def _button_rising_edge(self, buttons, prev_buttons, button_index):
        return self._button_value(buttons, button_index) == 1 and self._button_value(
            prev_buttons, button_index
        ) == 0

    def _reset_pid_state_locked(self, now):
        self.sum_err = [0.0] * 6
        self.prev_pose_err = [0.0] * 6
        self.prev_time = now

    def _handle_joy_input_locked(self, now):
        if not self.enable_joy or self.goal_pose is None or self.current_pose is None:
            return False

        axes = self.joy_axes
        buttons = self.joy_buttons
        prev_buttons = self.prev_joy_buttons

        if axes:
            x_input = self._axis_value(axes, self.joy_axis_forward, invert=True)
            y_input = self._axis_value(axes, self.joy_axis_lateral)
            yaw_input = self._axis_value(axes, self.joy_axis_yaw)

            self.goal_pose[0] += x_input * self.step_x * self.joy_axis_scale
            self.goal_pose[1] += y_input * self.step_y * self.joy_axis_scale
            self.goal_pose[5] = normalize_angle(
                self.goal_pose[5] + yaw_input * self.step_yaw * self.joy_axis_scale
            )

        if buttons:
            z_input = (
                self._button_value(buttons, self.joy_button_up)
                - self._button_value(buttons, self.joy_button_down)
            )
            self.goal_pose[2] += z_input * self.step_z * self.joy_axis_scale

            if self._button_rising_edge(buttons, prev_buttons, self.joy_button_hold):
                self.goal_pose = list(self.current_pose)
                self._reset_pid_state_locked(now)
                self.get_logger().info('Controller hold pressed. Holding current pose.')

            if self._button_rising_edge(buttons, prev_buttons, self.joy_button_quit):
                self.prev_joy_buttons = list(buttons)
                return True

        self.prev_joy_buttons = list(buttons)
        return False

    def publish_thrusters(self, thrusters):
        msg = Float64MultiArray()
        msg.data = thrusters
        self.publisher_.publish(msg)

    def publish_zero_thrusters(self):
        with self.state_lock:
            self.thruster_speeds = [0.0] * 8
        self.publish_thrusters([0.0] * 8)

    def timer_callback(self):
        now = time.monotonic()
        with self.state_lock:
            should_quit = self._handle_joy_input_locked(now)
            current_pose = None if self.current_pose is None else list(self.current_pose)
            goal_pose = None if self.goal_pose is None else list(self.goal_pose)
            quaternion_wxyz = self.current_quaternion
            sum_err = list(self.sum_err)
            prev_pose_err = list(self.prev_pose_err)
            prev_time = self.prev_time

        if should_quit:
            self.get_logger().info('Controller quit pressed. Shutting down.')
            self.running = False
            self.publish_zero_thrusters()
            rclpy.shutdown()
            return

        if current_pose is None or goal_pose is None or quaternion_wxyz is None:
            self.publish_zero_thrusters()
            if now - self.last_no_odom_warn >= 1.0:
                self.get_logger().warning('No odometry yet; publishing zero thrusters.')
                self.last_no_odom_warn = now
            return

        dt = 0.0 if prev_time is None else (now - prev_time)

        thrusters, pose_err, sum_err_next = compute_thruster_command(
            current_pose=current_pose,
            goal_pose=goal_pose,
            sum_err=sum_err,
            prev_pose_err=prev_pose_err,
            dt=dt,
            quaternion_wxyz=quaternion_wxyz,
            kp=self.kp,
            ki=self.ki,
            kd=self.kd,
            saturate=self.thruster_saturate,
        )

        with self.state_lock:
            self.sum_err = sum_err_next
            self.prev_pose_err = pose_err
            self.prev_time = now
            self.thruster_speeds = thrusters

        self.publish_thrusters(thrusters)


def main(args=None):
    rclpy.init(args=args)

    thruster_teleop = None
    try:
        thruster_teleop = ThrusterTeleOp()
        rclpy.spin(thruster_teleop)
    except KeyboardInterrupt:
        print('\nShutting down...')
    finally:
        if thruster_teleop is not None:
            thruster_teleop.running = False
            thruster_teleop.publish_zero_thrusters()
            thruster_teleop.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
