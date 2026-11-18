#!/usr/bin/env python3
"""Launch, exercise and shut down ONLY a Gazebo Scout in an isolated domain."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', default='clearpath_playpen')
    parser.add_argument('--no-camera', action='store_true')
    parser.add_argument('--data-only', action='store_true', help='Check RGB-D and TF without motion (slow software rendering)')
    args = parser.parse_args()
    if not 190 <= int(os.environ.get('ROS_DOMAIN_ID', '0')) <= 199:
        raise SystemExit('Use an isolated test domain 190–199 and a unique IGN_PARTITION')
    root = Path(__file__).resolve().parents[1]
    logdir = root / 'log/simulation'
    logdir.mkdir(parents=True, exist_ok=True)
    name = args.world.replace('/', '_')
    logfile = logdir / (name + ('_physics' if args.no_camera else '_rgbd') + '.log')
    rclpy.init()
    node = rclpy.create_node('simulation_smoke')
    seen, counts = {}, {}
    subscriptions = []
    topics = [('/clock', Clock), ('/scout/odom', Odometry),
              ('/scout_sim/ground_truth', Odometry), ('/joint_states', JointState)]
    if not args.no_camera:
        topics += [('/nav_camera/color/image_raw', Image), ('/nav_camera/depth/image_raw', Image),
                   ('/nav_camera/color/camera_info', CameraInfo), ('/nav_camera/depth/points', PointCloud2)]
    def receive(topic, msg):
        seen[topic] = msg
        counts[topic] = counts.get(topic, 0) + 1
    for topic, cls in topics:
        subscriptions.append(node.create_subscription(cls, topic,
            lambda msg, topic=topic: receive(topic, msg),
            QoSProfile(depth=2) if '/nav_camera/' in topic else qos_profile_sensor_data))
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    publisher = node.create_publisher(Twist, '/scout_sim/cmd_vel', 10)
    def sim_time():
        clock = seen['/clock'].clock
        return clock.sec + clock.nanosec * 1e-9
    def spin(seconds, cmd=None, simulated=False):
        start = sim_time() if simulated else time.monotonic()
        deadline = time.monotonic() + 60
        while (sim_time() if simulated else time.monotonic()) - start < seconds:
            if time.monotonic() > deadline:
                raise AssertionError('Simulation time stalled for 60 seconds; inspect renderer performance/logs')
            if cmd is not None:
                publisher.publish(cmd)
            rclpy.spin_once(node, timeout_sec=0.05)
    def position():
        p = seen['/scout_sim/ground_truth'].pose.pose.position
        return np.array([p.x, p.y, p.z])
    launch = None
    try:
        with logfile.open('w') as output:
            launch = subprocess.Popen(['ros2', 'launch', 'scout_sim', 'simulation.launch.py',
                'world:=' + args.world, 'gui:=false', 'software_rendering:=true',
                'camera_pitch_deg:=20', 'camera:=' + ('false' if args.no_camera else 'true')],
                stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
            end = time.monotonic() + 50
            while len(seen) < len(topics) and time.monotonic() < end:
                if launch.poll() is not None:
                    raise RuntimeError(f'Simulator exited; inspect {logfile}')
                rclpy.spin_once(node, timeout_sec=0.1)
            missing = [topic for topic, _ in topics if topic not in seen]
            if missing:
                topics_debug = subprocess.run(['ign', 'topic', '-l'], capture_output=True, text=True, timeout=5)
                print('Gazebo topics:\n' + topics_debug.stdout, flush=True)
                print('Received message counts: ' + json.dumps(counts), flush=True)
                cloud_debug = subprocess.run(['bash', '-c',
                    'timeout 3 ign topic -e -t /scout_sim/camera/points | head -c 1000'],
                    capture_output=True, text=True, timeout=5)
                print('Native point cloud sample: ' + cloud_debug.stdout, flush=True)
                raise AssertionError(f'No data on {missing}; inspect {logfile}')
            if not args.no_camera:
                import cv2
                color = seen['/nav_camera/color/image_raw']
                rgb = np.frombuffer(color.data, np.uint8).reshape(color.height, color.width, -1)
                image = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR) if color.encoding == 'rgb8' else rgb
                cv2.imwrite(str(logdir / (name + '_color.png')), image)
            spin(1, simulated=not args.data_only)
            before = position()
            initial_clock = seen['/clock'].clock
            initial_time = initial_clock.sec + initial_clock.nanosec * 1e-9
            move = Twist()
            move.linear.x = .12 if not args.data_only else 0.0
            if not args.data_only:
                spin(4, move, simulated=True)
                spin(1.5, simulated=True)  # Stop publishing: must trigger the command watchdog.
            after = position()
            displacement = float(np.linalg.norm(after[:2] - before[:2]))
            assert args.data_only or displacement > .05, f'Wheels turned but chassis did not move: {displacement}; counts={counts}'
            assert abs(seen['/scout/odom'].twist.twist.linear.x) < .02, 'Command watchdog did not stop wheel odometry'
            stopped = position()
            if not args.data_only:
                spin(.5, simulated=True)
            assert np.linalg.norm(position()[:2] - stopped[:2]) < .02, 'Chassis kept moving after timeout'
            yaw_before = seen['/scout_sim/ground_truth'].pose.pose.orientation
            turn = Twist()
            turn.angular.z = .3
            if not args.data_only:
                spin(3, turn, simulated=True)
                spin(1.2, simulated=True)
            yaw_after = seen['/scout_sim/ground_truth'].pose.pose.orientation
            def yaw(q):
                return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            yaw_delta = math.atan2(math.sin(yaw(yaw_after)-yaw(yaw_before)),
                                   math.cos(yaw(yaw_after)-yaw(yaw_before)))
            print(json.dumps({'forward_m': displacement, 'turn_rad': yaw_delta,
                              'odom_yaw': yaw(seen['/scout/odom'].pose.pose.orientation),
                              'joints': dict(zip(seen['/joint_states'].name, seen['/joint_states'].position))}), flush=True)
            assert args.data_only or yaw_delta > .05, f'Positive angular command did not turn chassis left: {yaw_delta}'
            transform = buffer.lookup_transform('base_link', 'nav_camera_color_optical_frame', rclpy.time.Time())
            assert transform.transform.translation.z > .6
            assert seen['/scout/odom'].header.frame_id == 'odom'
            assert seen['/scout/odom'].child_frame_id == 'base_link'
            assert len(seen['/joint_states'].name) == 4
            result = {'world': args.world, 'camera': not args.no_camera, 'counts': counts,
                      'forward_displacement_m': displacement, 'left_turn_radians': yaw_delta,
                      'motion_tested': not args.data_only,
                      'timeout_stop': not args.data_only, 'optical_tf': True, 'initial_sim_time': initial_time,
                      'log': str(logfile)}
            if not args.no_camera:
                color, depth = seen['/nav_camera/color/image_raw'], seen['/nav_camera/depth/image_raw']
                info, points = seen['/nav_camera/color/camera_info'], seen['/nav_camera/depth/points']
                for msg in (color, depth, info):
                    assert msg.header.frame_id == 'nav_camera_color_optical_frame'
                    assert (msg.width, msg.height) == (640, 360)
                assert points.header.frame_id == 'nav_camera_link'
                assert (points.width, points.height) == (640, 360)
                assert depth.encoding == '32FC1'
                values = np.frombuffer(depth.data, dtype='<f4')
                assert np.isfinite(values).sum() > 100, 'No finite depth measurements'
                assert info.k[0] > 0
                result.update(color_encoding=color.encoding, depth_encoding=depth.encoding,
                              depth_finite_pixels=int(np.isfinite(values).sum()))
                import struct
                offsets = {f.name: f.offset for f in points.fields}
                center = points.row_step * (points.height // 2) + points.point_step * (points.width // 2)
                xyz = [struct.unpack_from('<f', points.data, center + offsets[k])[0] for k in ('x', 'y', 'z')]
                result['cloud_center_xyz'] = xyz
                # Fortress native point data uses camera_link X-forward coordinates.
                if all(math.isfinite(v) for v in xyz):
                    assert xyz[0] > 0 and abs(xyz[1]) < .05 and abs(xyz[2]) < .05
                # Save actual simulator image for inspection; no fabricated preview.
                import cv2
                rgb = np.frombuffer(color.data, np.uint8).reshape(color.height, color.width, -1)
                image = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR) if color.encoding == 'rgb8' else rgb
                cv2.imwrite(str(logdir / (name + '_color.png')), image)
            (logdir / (name + '_result.json')).write_text(json.dumps(result, indent=2) + '\n')
            print(json.dumps(result, indent=2))
    finally:
        publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
        if launch is not None:
            try:
                launch.send_signal(signal.SIGINT)
                launch.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGTERM)
                try:
                    launch.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(launch.pid, signal.SIGKILL)
                    launch.wait()
            except ProcessLookupError:
                pass


if __name__ == '__main__':
    main()
