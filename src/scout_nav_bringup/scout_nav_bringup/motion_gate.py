"""One explicitly selected command source, freshness checks and manual enable."""
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool


class CommandPolicy:
    def __init__(self, mode='disabled', timeout=0.4, sensor_timeout=0.6,
                 max_linear=0.15, max_angular=0.35):
        if mode not in ('disabled', 'navigation', 'follow'):
            raise ValueError('mode must be disabled, navigation or follow')
        if not all(math.isfinite(v) and v > 0
                   for v in (timeout, sensor_timeout, max_linear, max_angular)):
            raise ValueError('Timeouts and speed limits must be finite and positive')
        self.mode, self.timeout, self.sensor_timeout = mode, timeout, sensor_timeout
        self.max_linear, self.max_angular = max_linear, max_angular
        self.enabled, self.command, self.command_time = False, (0.0, 0.0), None
        self.depth_time, self.odom_time = None, None

    def enable(self, enabled):
        self.enabled = bool(enabled) and self.mode != 'disabled'
        self.command_time = None  # Never replay commands from before enable.
        self.command = (0.0, 0.0)

    def receive(self, source, vx, wz, now):
        if source != self.mode:
            return
        if not all(math.isfinite(v) for v in (vx, wz)):
            self.enable(False)
            return
        self.command = (max(0.0, min(self.max_linear, vx)),
                        max(-self.max_angular, min(self.max_angular, wz)))
        self.command_time = now

    def output(self, now):
        if not self.enabled:
            return 0.0, 0.0
        sensor_fresh = all(t is not None and 0 <= now - t <= self.sensor_timeout
                           for t in (self.depth_time, self.odom_time))
        if not sensor_fresh:
            self.enable(False)  # Latch off; recovering camera data does not resume motion.
            return 0.0, 0.0
        if self.command_time is None:
            return 0.0, 0.0
        if not 0 <= now - self.command_time <= self.timeout:
            self.enable(False)
            return 0.0, 0.0
        return self.command


class MotionGate(Node):
    def __init__(self):
        super().__init__('motion_gate', namespace='scout')
        defaults = dict(mode='disabled', command_timeout=0.4, sensor_timeout=0.6,
                        max_linear_speed=0.15, max_angular_speed=0.35,
                        depth_topic='/nav_camera/depth/image_raw', odom_topic='/scout/odom',
                        auto_enable=False)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        param = lambda key: self.get_parameter(key).value
        self.policy = CommandPolicy(param('mode'), param('command_timeout'), param('sensor_timeout'),
                                    param('max_linear_speed'), param('max_angular_speed'))
        self.startup_enable_pending = bool(param('auto_enable'))
        self.lock = threading.RLock()
        self.output_pub = self.create_publisher(Twist, '/scout/cmd_vel', 1)
        self.sources = [self.create_subscription(Twist, f'/scout/{source}_cmd_vel',
                        lambda msg, source=source: self.receive(source, msg), 1)
                        for source in ('navigation', 'follow')]
        self.depth_sub = self.create_subscription(Image, param('depth_topic'),
                                                  self.depth, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(Odometry, param('odom_topic'), self.odom, 5)
        self.enable_srv = self.create_service(SetBool, 'motion_enable', self.enable)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.05, self.tick, clock=self.steady_clock)
        self.get_logger().info(f'Command mode={self.policy.mode}; motion DISABLED. ' +
                               ('Startup enable waits for fresh depth, odom and command; runs ONCE.'
                                if self.startup_enable_pending else
                                'Explicit /scout/motion_enable service call required.'))

    def receive(self, source, msg):
        with self.lock:
            components = [msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y]
            if any(not math.isfinite(v) or abs(v) > 1e-6 for v in components):
                self.policy.enable(False)
                return
            self.policy.receive(source, msg.linear.x, msg.angular.z, time.monotonic())

    def stamp_fresh(self, msg):
        stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        return stamp > 0 and -0.08 <= age <= self.policy.sensor_timeout

    def depth(self, msg):
        with self.lock:
            self.policy.depth_time = time.monotonic() if self.stamp_fresh(msg) else None

    def odom(self, msg):
        with self.lock:
            self.policy.odom_time = time.monotonic() if self.stamp_fresh(msg) else None

    def enable(self, req, res):
        with self.lock:
            self.startup_enable_pending = False
            now = time.monotonic()
            ready = all(t is not None and 0 <= now - t <= self.policy.sensor_timeout
                        for t in (self.policy.depth_time, self.policy.odom_time))
            accepted = not req.data or (ready and self.policy.mode != 'disabled')
            self.policy.enable(req.data and accepted)
            res.success = accepted
            res.message = ('Enabled; awaiting a NEW command' if self.policy.enabled else
                           'Disabled' if not req.data else 'Need fresh depth/odom and a selected mode')
            self.output_pub.publish(Twist())
        return res

    def tick(self):
        with self.lock:
            now = time.monotonic()
            if (self.startup_enable_pending and self.policy.mode != 'disabled'
                    and all(t is not None and 0 <= now - t <= self.policy.sensor_timeout
                            for t in (self.policy.depth_time, self.policy.odom_time))
                    and self.policy.command_time is not None
                    and 0 <= now - self.policy.command_time <= self.policy.timeout):
                self.startup_enable_pending = False
                self.policy.enable(True)
                self.get_logger().info('Startup motion enable complete; awaiting a NEW command. '
                                       'Any timeout latches off until manual enable.')
            was_enabled = self.policy.enabled
            vx, wz = self.policy.output(now)
            msg = Twist()
            msg.linear.x, msg.angular.z = vx, wz
            self.output_pub.publish(msg)
            if was_enabled and not self.policy.enabled:
                self.get_logger().warning('Stale input: stopped and disabled; explicit enable required again.')

    def stop(self):
        with self.lock:
            self.startup_enable_pending = False
            self.policy.enable(False)
            self.output_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = MotionGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        # Humble may raise while taking a subscription as SIGINT tears down
        # the context. Do not hide errors while the context is still running.
        if rclpy.ok():
            raise
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
