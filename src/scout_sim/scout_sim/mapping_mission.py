"""Drive the fixed simulation course, yielding to a confirmed RGB-D target."""
import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.clock import Clock
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

COURSE = ((2.4, 0.), (2.4, 1.8), (0., 1.8), (0., 0.), (.32, 0.))
MAX_LAPS = 2


def simulation_domain():
    return os.environ.get('ROS_DOMAIN_ID') in {'42', *(str(n) for n in range(190, 200))}


def angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def clamp(value, limit):
    return max(-limit, min(value, limit))


def fresh_vision(now, sim_seconds, received, image_stamp):
    return received is not None and now - received < .6 and -.08 <= sim_seconds - image_stamp <= .6


class Mission:
    """The route advances only while mapping owns the motion output."""
    def __init__(self):
        self.start = None
        self.waypoint = self.laps = 0
        self.closure = None
        self.departed = False
        self.tracking = False
        self.resume_at = 0.

    def record_closure(self, pose, source, match):
        if (self.closure is None and self.departed and source > 0 and match > 0
                and self.start and self.waypoint >= 3
                and math.dist(pose[:2], self.start[:2]) < .7
                and abs(angle(pose[2] - self.start[2])) < .60):
            self.closure = (source, match)

    def step(self, now, pose, vision, ready):
        if not ready:
            self.tracking = False
            self.resume_at = now + .5
            return 'waiting_for_inputs', 0., 0.
        if self.start is None:
            self.start = pose
        if vision and vision['selected_id'] is not None:
            valid = vision['control_state'] in ('following', 'distance_hold')
            if not self.tracking:
                if not valid:
                    return 'target_lost', 0., 0.
                self.tracking = True
                return 'target_acquired', 0., 0.
            if not valid:
                return 'target_lost', 0., 0.
            if vision['control_state'] == 'distance_hold':
                return 'distance_hold', 0., 0.
            linear, angular = vision['command']
            return 'following', max(0., min(linear, .14)), clamp(angular, .35)
        if self.tracking:
            self.tracking = False
            self.resume_at = now + .5
        if now < self.resume_at:
            return 'stopping_before_mapping', 0., 0.
        if self.laps and self.closure:
            return 'loop_closed', 0., 0.
        if self.laps >= MAX_LAPS:
            return 'loop_not_confirmed', 0., 0.
        sx, sy, start_yaw = self.start
        x, y, heading = pose
        self.departed |= math.hypot(x - sx, y - sy) > 1.4
        dx, dy = COURSE[self.waypoint]
        cx, sn = math.cos(start_yaw), math.sin(start_yaw)
        gx, gy = sx + dx * cx - dy * sn, sy + dx * sn + dy * cx
        distance = math.hypot(gx - x, gy - y)
        if distance < .14:
            self.waypoint += 1
            if self.waypoint == len(COURSE):
                self.laps += 1
                self.waypoint = 0
            return 'waypoint_reached', 0., 0.
        error = angle(math.atan2(gy - y, gx - x) - heading)
        linear = 0. if abs(error) > .16 else min(.25, max(.09, distance)) * math.cos(error)
        return 'mapping', linear, clamp(error * 1.8, .5)


class MappingMission(Node):
    def __init__(self):
        super().__init__('mapping_mission', namespace='scout_sim')
        if not simulation_domain() or not self.get_parameter('use_sim_time').value:
            raise RuntimeError('Mapping mission requires sim_env.bash and use_sim_time:=true')
        from rtabmap_msgs.msg import Info

        self.declare_parameter('vision_enabled', True)
        self.vision_enabled = self.get_parameter('vision_enabled').value
        self.mission = Mission()
        self.last_odom = self.last_slam = self.last_vision = None
        self.vision = None
        self.grid_ready = False
        self.last_state = None
        self.last_status = 0.
        self.pending_closure = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.odom_pub = self.create_publisher(Odometry, '/scout_sim/mapping_odom', 10)
        self.odom_tf = TransformBroadcaster(self)
        self.subscriptions_keepalive = [
            self.create_subscription(Odometry, '/scout_sim/ground_truth', self.odometry, 5),
            self.create_subscription(Info, '/scout/rtabmap/info', self.slam, 5),
            self.create_subscription(OccupancyGrid, '/map', self.map_update,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        ]
        if self.vision_enabled:
            self.subscriptions_keepalive.append(self.create_subscription(
                String, '/scout/follow_targets', self.target, 5))
        self.commands = self.create_publisher(Twist, '/scout_sim/cmd_vel', 1)
        self.status = self.create_publisher(String, '/scout_sim/mapping_state', 1)
        self.create_timer(.05, self.tick, clock=Clock())  # stop even if /clock pauses

    def odometry(self, msg):
        if msg.header.frame_id != 'world' or msg.child_frame_id != 'base_link':
            return
        pose = msg.pose.pose
        if not all(math.isfinite(v) for v in (pose.position.x, pose.position.y, yaw(pose.orientation))):
            return
        # Explicit ideal odometry for this simulation benchmark. Keep wheel odom
        # available on its original topic for independent comparison.
        msg.header.frame_id = 'mapping_odom'
        t = TransformStamped(header=msg.header, child_frame_id='base_link')
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation = pose.orientation
        self.odom_tf.sendTransform(t)
        self.odom_pub.publish(msg)
        self.last_odom = time.monotonic(), Time.from_msg(msg.header.stamp).nanoseconds

    def slam(self, msg):
        self.last_slam = time.monotonic(), Time.from_msg(msg.header.stamp).nanoseconds
        if msg.loop_closure_id > 0:
            self.pending_closure = (msg.ref_id, msg.loop_closure_id, msg.header.stamp)

    def map_update(self, msg):
        self.grid_ready = msg.info.width > 0 and msg.info.height > 0

    def target(self, msg):
        try:
            v = json.loads(msg.data)
            linear, angular = v['command']
            if (not math.isfinite(linear) or not math.isfinite(angular)
                    or not math.isfinite(v['image_stamp']) or 'selected_id' not in v
                    or 'control_state' not in v):
                return
        except (TypeError, ValueError, KeyError):
            return
        self.last_vision, self.vision = time.monotonic(), v

    def pose(self, stamp=Time()):
        try:
            t = self.buffer.lookup_transform('map', 'base_link', stamp).transform
            p = (t.translation.x, t.translation.y, yaw(t.rotation))
            return p if all(map(math.isfinite, p)) else None
        except TransformException:
            return None

    def tick(self):
        now, sim = time.monotonic(), self.get_clock().now().nanoseconds
        pose = self.pose()
        vision_ok = (not self.vision_enabled or (self.vision is not None and fresh_vision(
            now, sim * 1e-9, self.last_vision, self.vision['image_stamp'])))
        odom_ok = (self.last_odom is not None and now - self.last_odom[0] < .45
                   and 0 <= sim - self.last_odom[1] < 450_000_000)
        slam_ok = self.last_slam is not None and now - self.last_slam[0] < 5.
        ready = bool(sim and pose and vision_ok and odom_ok and slam_ok and self.grid_ready)
        if self.pending_closure and pose:
            source, match, stamp = self.pending_closure
            match_pose = self.pose(Time.from_msg(stamp))
            if match_pose is not None:
                self.mission.record_closure(match_pose, source, match)
                self.pending_closure = None
            elif sim - Time.from_msg(stamp).nanoseconds > 2_000_000_000:
                self.pending_closure = None
        state, linear, angular = self.mission.step(now, pose, self.vision, ready)
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = linear, angular
        self.commands.publish(cmd)
        changed = state != self.last_state
        if changed and state != 'mapping':
            self.get_logger().info(f'{state}: lap={self.mission.laps} waypoint={self.mission.waypoint} '
                                   f'closure={self.mission.closure}')
        self.last_state = state
        if changed or now - self.last_status > .4:
            self.status.publish(String(data=json.dumps(dict(
                state=state, pose=pose, completed_laps=self.mission.laps,
                waypoint=self.mission.waypoint, closure=self.mission.closure,
                command=[linear, angular], vision_ready=bool(vision_ok), odom_ready=bool(odom_ok),
                slam_ready=bool(slam_ok), map_ready=self.grid_ready))))
            self.last_status = now


def main():
    rclpy.init()
    node = None
    try:
        node = MappingMission()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.commands.publish(Twist())
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
