#!/usr/bin/env python3
"""Verify the optional beginner Gazebo activities in the sourced ROS container.

Uses domain 89 and a separate Gazebo partition by default. --gui opens RViz,
rqt Plot and Node Graph on the container desktop; inspect the saved screenshot.
--reuse uses an already-running simulation in the chosen domain without closing it.
Only process groups launched by this verifier are stopped.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from PIL import Image as PILImage
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan

from intro_ros import IntroLab, front_distance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--reuse', action='store_true')
    parser.add_argument('--domain', type=int, default=89)
    parser.add_argument('--output', type=Path, default=Path('/tmp/intro-gazebo-verification'))
    args = parser.parse_args()
    if not 0 <= args.domain <= 232:
        parser.error('--domain must be between 0 and 232')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['GZ_PARTITION'] = f'ros_fun_intro_validation_{args.domain}'
    exercises = Path(__file__).resolve().parent
    args.output.mkdir(parents=True, exist_ok=True)
    processes = []

    def launch(name, command):
        log = args.output / f'{name}.log'
        with log.open('w') as stream:
            process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                       start_new_session=True)
        processes.append((name, process, log))

    def check_processes():
        for name, process, log in processes:
            assert process.poll() is None, f'{name} exited: see {log}'
            errors = ('No usable plot type found', '_ARRAY_API not found',
                      'numpy.core.multiarray failed to import', 'Traceback (most recent call last)')
            assert not any(error in log.read_text() for error in errors), f'{name}: see {log}'

    lab = None
    try:
        lab = IntroLab('beginner_gazebo_verifier')
        if not args.reuse:
            lab.spin_for(5)
            existing = lab.node.get_node_names_and_namespaces()
            active = [(name, namespace) for name, namespace in existing
                      if not name.startswith('_ros2cli_daemon_')]
            assert active == [('beginner_gazebo_verifier', '/')], (
                f'Domain {args.domain} is in use; choose an unused --domain', existing)
            launch('gazebo', ['ros2', 'launch', 'turtlebot3_gazebo', 'turtlebot3_world.launch.py'])
        node = lab.node
        state = dict.fromkeys(('odom', 'image', 'scan'))
        counts = dict.fromkeys(state, 0)

        def collect(key, message):
            state[key] = message
            counts[key] += 1

        for key, message_type, topic in (
            ('odom', Odometry, '/odom'), ('image', Image, '/camera/image_raw'),
            ('scan', LaserScan, '/scan'),
        ):
            node.create_subscription(message_type, topic,
                                     lambda message, key=key: collect(key, message),
                                     qos_profile_sensor_data)
        node.set_parameters([Parameter('use_sim_time', value=True)])
        lab.wait_for(lambda: node.get_clock().now().nanoseconds > 0,
                     timeout=90, description='Gazebo clock')
        lab.wait_for(lambda: all(message is not None for message in state.values()),
                     timeout=60, description='odom/image/scan')
        check_processes()
        image = state['image']
        assert image.width > 0 and image.height > 0
        assert len(image.data) == image.height * image.step
        assert len(set(image.data)) > 10, 'Camera image is blank'
        assert image.encoding in ('rgb8', 'bgr8'), image.encoding
        pixels = PILImage.frombytes('RGB', (image.width, image.height), bytes(image.data),
                                   'raw', 'BGR' if image.encoding == 'bgr8' else 'RGB', image.step)
        pixels.save(args.output / 'camera.png')
        print('CAMERA_PASS', image.width, image.height, image.encoding, flush=True)

        # Execute the lesson cell itself, including its actual subscriber and QoS.
        notebook = json.loads((exercises / '4. ROS Topic - Subscriber.ipynb').read_text())
        source = next(''.join(cell['source']) for cell in notebook['cells']
                      if cell['cell_type'] == 'code' and 'RUN_LASER = False' in ''.join(cell['source']))
        scope = {'lab': lab, 'node': node}
        exec(source.replace('RUN_LASER = False', 'RUN_LASER = True'), scope)
        assert scope['radar']['received']
        scan = state['scan']
        distance = front_distance(scan)
        assert distance is not None and scan.range_min <= distance <= scan.range_max
        print('LASER_NOTEBOOK_PASS', len(scan.ranges), 'front', distance, flush=True)

        if args.gui:
            launch('rviz', ['ros2', 'run', 'rviz2', 'rviz2', '-d',
                            str(exercises.parent / 'turtlebot_kamera.rviz')])
            launch('plot', ['ros2', 'run', 'rqt_plot', 'rqt_plot', '/odom/pose/pose/position/x'])
            launch('graph', ['ros2', 'run', 'rqt_graph', 'rqt_graph'])
            lab.spin_for(20)
            check_processes()

        publisher = node.create_publisher(TwistStamped, '/cmd_vel', 10)
        lab.watch_motion(publisher)
        lab.wait_for_subscriber(publisher)
        origin = state['odom'].pose.pose.position
        started = node.get_clock().now().nanoseconds / 1e9
        wall = time.monotonic()
        deadline = wall + 120
        while node.get_clock().now().nanoseconds / 1e9 - started < 2:
            if time.monotonic() > deadline:
                raise TimeoutError('Gazebo clock is too slow for the motion check')
            lab.velocity(publisher, linear=0.1)
            lab.spin_for(0.1)
        stopped = node.get_clock().now().nanoseconds / 1e9
        deadline = time.monotonic() + 30
        while node.get_clock().now().nanoseconds / 1e9 - stopped < 0.3:
            if time.monotonic() > deadline:
                raise TimeoutError('Gazebo clock is too slow for the stop check')
            lab.velocity(publisher)
            lab.spin_for(0.1)
        position = state['odom'].pose.pose.position
        travel = math.hypot(position.x - origin.x, position.y - origin.y)
        velocity = state['odom'].twist.twist.linear.x
        assert travel > 0.05, f'Robot did not move: {travel}'
        assert abs(velocity) < 0.01, f'Robot did not stop: {velocity}'
        result = {'image': [image.width, image.height, image.encoding],
                  'laser_samples': len(scan.ranges), 'front_distance': distance,
                  'travel_m': travel, 'final_linear_velocity': velocity,
                  'messages': counts, 'motion_wall_seconds': time.monotonic() - wall,
                  'motion_sim_seconds': node.get_clock().now().nanoseconds / 1e9 - started,
                  'gui_processes_checked': args.gui, 'reused_simulation': args.reuse}
        print('GAZEBO_MOTION_STOP_PASS', result, flush=True)
        if args.gui:
            lab.spin_for(2)
            check_processes()
            # Capture the actual desktop when ImageMagick is installed. Rendering
            # still needs human inspection; a running GUI alone does not prove it.
            capture = subprocess.run(['import', '-window', 'root',
                                      str(args.output / 'desktop.png')], timeout=20,
                                     capture_output=True)
            assert capture.returncode == 0, capture.stderr.decode(errors='replace')
            print('GUI_PROCESSES_PASS: inspect desktop.png', flush=True)
        (args.output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    finally:
        try:
            if lab is not None:
                lab.close()
        finally:
            for _, process, _ in reversed(processes):
                for sig, timeout in ((signal.SIGINT, 5), (signal.SIGTERM, 3), (signal.SIGKILL, 3)):
                    try:
                        os.killpg(process.pid, sig)
                    except ProcessLookupError:
                        break
                    try:
                        process.wait(timeout=timeout)
                    except subprocess.TimeoutExpired:
                        continue
                    # A launcher's shell can exit before its grandchildren do.
                    # Continue signalling the owned group to clean those up too.


if __name__ == '__main__':
    main()
