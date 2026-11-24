"""Reserve the Scout ROS domain and reject a pre-existing simulation clock."""
import fcntl
import os
from pathlib import Path
import subprocess
import time

import rclpy
from rclpy.context import Context
from rclpy.signals import SignalHandlerOptions


def reserve(domain, partition, lock_dir=Path('/tmp')):
    # A shared path prevents different shell TMPDIR values bypassing the lock.
    lock = (lock_dir / f'scout-sim-{os.getuid()}-{domain}.lock').open('a')
    try:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError(f'Scout simulation is already running in ROS domain {domain}. '
                               'Stop it first or choose a different SCOUT_SIM_DOMAIN.') from None
        check_existing_clock(domain, partition)
    except BaseException:
        lock.close()
        raise
    return lock


def check_existing_clock(domain, partition):
    context = Context()
    rclpy.init(args=[], context=context, domain_id=domain, signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = rclpy.create_node('scout_sim_clock_check', context=context,
                                 enable_rosout=False, start_parameter_services=False)
        try:
            # DDS discovery is asynchronous, including for a bridge left by an old launch.
            deadline = time.monotonic() + 1.5
            while True:
                publishers = node.get_publishers_info_by_topic('/clock')
                if publishers:
                    names = ', '.join(f'{p.node_namespace.rstrip("/")}/{p.node_name}' for p in publishers)
                    raise RuntimeError(f'ROS domain {domain} already has a /clock publisher ({names}). '
                                       'Stop the old simulation or choose a different SCOUT_SIM_DOMAIN.')
                if time.monotonic() >= deadline:
                    break
                time.sleep(.05)
        finally:
            node.destroy_node()
    finally:
        context.try_shutdown()

    reply = subprocess.run(['ign', 'topic', '-i', '-t', '/world/scout_world/clock'], check=True,
                           capture_output=True, text=True, timeout=5,
                           env={**os.environ, 'IGN_PARTITION': partition})
    publishers = reply.stdout.partition('Publishers [')[2].partition('Subscribers [')[0]
    if 'ignition.msgs.Clock' in publishers:
        raise RuntimeError(f'Gazebo partition {partition} already has a simulation clock. '
                           'Stop the old Gazebo instance or choose a different SCOUT_SIM_DOMAIN.')
