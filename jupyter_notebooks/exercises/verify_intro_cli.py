#!/usr/bin/env python3
"""Exercise the beginner workshop's terminal activities against real ROS nodes.

Run as the desktop/Jupyter user in the sourced workshop container:
    python3 verify_intro_cli.py

The default isolated ROS domain is 92. Only owned processes are stopped; an
already running turtlesim in that domain causes an error instead of being moved.
Keyboard input uses a controlling pseudo-terminal, including actual Ctrl+C.
Gazebo, camera rendering and human assessment of drawings are separate checks.
"""
import argparse
from contextlib import contextmanager
import fcntl
import json
import math
import os
from pathlib import Path
import pty
import signal
import subprocess
import termios


def stop_process(process):
    """Stop a process group created by this script, escalating with deadlines."""
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            break
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            pass
        # A ros2 launcher can exit while its descendants remain in the group.
        # Continue escalation until killpg reports that the owned group is gone.


@contextmanager
def cli_process(command):
    process = subprocess.Popen(command, stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE, text=True,
                               start_new_session=True)
    try:
        yield process
    finally:
        stop_process(process)


def run_cli(command, expected=None, timeout=12):
    with cli_process(command) as process:
        output, error = process.communicate(timeout=timeout)
        assert process.returncode == 0, (command, output, error)
        if expected is not None:
            assert expected in output, (command, expected, output, error)
        return output


@contextmanager
def keyboard_process(command):
    master, slave = pty.openpty()
    process = None
    try:
        # ISIG needs a controlling terminal and foreground process group for
        # Ctrl+C to behave as it does in the student's GUI terminal.
        process = subprocess.Popen(
            command, stdin=slave, stdout=slave, stderr=slave,
            start_new_session=True,
            preexec_fn=lambda: fcntl.ioctl(0, termios.TIOCSCTTY, 0))
        os.close(slave)
        slave = None
        yield process, master
    finally:
        if process is not None:
            stop_process(process)
        if slave is not None:
            os.close(slave)
        os.close(master)


def angle_difference(current, origin):
    return math.atan2(math.sin(current - origin), math.cos(current - origin))


def heading_displacement(current, origin):
    return ((current.x - origin.x) * math.cos(origin.theta)
            + (current.y - origin.y) * math.sin(origin.theta))


def check_turtle_keyboard(lab, state, passed):
    with keyboard_process(['ros2', 'run', 'turtlesim', 'turtle_teleop_key']) as (process, keyboard):
        lab.wait_for(lambda: any(name == 'teleop_turtle' for name, _ in
                                lab.node.get_node_names_and_namespaces()),
                     timeout=8, description='turtle teleop node')
        lab.spin_for(0.5)
        origin = state['pose']
        os.write(keyboard, b'\x1b[A')
        lab.wait_for(lambda: heading_displacement(state['pose'], origin) > 0.1,
                     timeout=2, description='up-arrow motion')
        lab.spin_for(1.3)
        stopped = state['pose']
        lab.spin_for(0.4)
        assert math.hypot(state['pose'].x - stopped.x,
                          state['pose'].y - stopped.y) < 0.02
        passed('Turtlesim: up-arrow moves; releasing it stops motion')

        theta = state['pose'].theta
        os.write(keyboard, b'\x1b[D')
        lab.wait_for(lambda: angle_difference(state['pose'].theta, theta) > 0.1,
                     timeout=2, description='left-arrow rotation')
        passed('Turtlesim: left-arrow rotates counterclockwise')
        # Let the previous rotation expire before measuring reverse motion
        # along the robot's new heading rather than a world-axis coordinate.
        lab.spin_for(1.3)
        assert abs(state['pose'].angular_velocity) < 0.01
        origin = state['pose']
        os.write(keyboard, b'\x1b[B')
        lab.wait_for(lambda: heading_displacement(state['pose'], origin) < -0.1,
                     timeout=2, description='down-arrow reverse motion')
        lab.spin_for(1.3)
        assert abs(state['pose'].linear_velocity) < 0.01
        passed('Turtlesim: down-arrow moves backward along its heading')

        theta = state['pose'].theta
        os.write(keyboard, b'\x1b[C')
        lab.wait_for(lambda: angle_difference(state['pose'].theta, theta) < -0.1,
                     timeout=2, description='right-arrow rotation')
        lab.spin_for(1.3)
        assert abs(state['pose'].angular_velocity) < 0.01
        passed('Turtlesim: right-arrow rotates clockwise')
        os.write(keyboard, b'\x03')
        process.wait(timeout=5)
        passed('Turtlesim: Ctrl+C exits teleop')


def check_documented_cli(lab, state, passed):
    commands = [
        (['ros2', 'node', 'list'], '/turtlesim'),
        (['ros2', 'topic', 'list', '-t'], '/turtle1/pose [turtlesim/msg/Pose]'),
        (['ros2', 'topic', 'info', '/turtle1/cmd_vel', '--verbose'], 'geometry_msgs/msg/Twist'),
        (['ros2', 'topic', 'echo', '/turtle1/pose', '--once'], 'theta:'),
        (['ros2', 'interface', 'show', 'geometry_msgs/msg/Twist'], 'angular'),
        (['ros2', 'service', 'list', '-t'], '/turtle1/set_pen [turtlesim/srv/SetPen]'),
        (['ros2', 'service', 'type', '/turtle1/set_pen'], 'turtlesim/srv/SetPen'),
        (['ros2', 'interface', 'show', 'turtlesim/srv/SetPen'], 'uint8 width'),
    ]
    for command, expected in commands:
        run_cli(command, expected)
        passed(' '.join(command))

    lab.spin_for(0.2)
    origin = state['pose']
    run_cli(['ros2', 'topic', 'pub', '--once', '/turtle1/cmd_vel',
             'geometry_msgs/msg/Twist', '{linear: {x: 1.0}, angular: {z: 1.0}}'])
    lab.spin_for(0.2)
    pose = state['pose']
    assert math.hypot(pose.x - origin.x, pose.y - origin.y) > 0.1
    assert abs(angle_difference(pose.theta, origin.theta)) > 0.1
    passed('Terminal --once velocity command produces an observed arc')

    from rcl_interfaces.srv import GetParameters
    response = lab.call(GetParameters, '/turtlesim/get_parameters',
                        GetParameters.Request(names=['use_sim_time']))
    assert len(response.values) == 1 and response.values[0].type == 1
    assert response.values[0].bool_value is False
    passed('Parameter bonus: turtlesim use_sim_time=False')


def check_external_clients(lab, passed):
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    publisher = lab.node.create_publisher(String, '/radio_zespol_1', 10)
    try:
        with cli_process(['ros2', 'topic', 'echo', '/radio_zespol_1',
                          'std_msgs/msg/String', '--once']) as process:
            lab.wait_for_subscriber(publisher, timeout=10)
            publisher.publish(String(data='tajne haslo'))
            output, error = process.communicate(timeout=8)
            assert process.returncode == 0 and 'tajne haslo' in output, (output, error)
        passed('Radio: external CLI subscriber receives the paired password')
    finally:
        lab.node.destroy_publisher(publisher)

    def receive_delivery(request, response):
        response.success = True
        response.message = 'Paczka odebrana!'
        return response

    # The notebook runner checks delivery-game eligibility. This probe checks
    # that the documented external terminal client reaches a spinning server.
    server = lab.node.create_service(Trigger, '/kurier/odbior', receive_delivery)
    try:
        with cli_process(['ros2', 'service', 'call', '/kurier/odbior',
                          'std_srvs/srv/Trigger', '{}']) as process:
            lab.wait_for(lambda: process.poll() is not None, timeout=12,
                         description='external service response')
            output, error = process.communicate(timeout=1)
            assert process.returncode == 0 and 'success=True' in output, (output, error)
            assert 'Paczka odebrana!' in output, output
        passed('External service CLI receives the Trigger server response')
    finally:
        lab.node.destroy_service(server)


def check_turtlebot_keyboard(lab, passed):
    from geometry_msgs.msg import TwistStamped

    messages = []
    subscription = lab.node.create_subscription(TwistStamped, '/cmd_vel', messages.append, 10)
    try:
        with keyboard_process(['ros2', 'run', 'turtlebot3_teleop', 'teleop_keyboard']) as (process, keyboard):
            lab.wait_for(lambda: bool(messages), timeout=8, description='TurtleBot teleop publisher')
            keys = [
                (b'w', 'forward', lambda msg: msg.twist.linear.x > 0),
                (b'a', 'left', lambda msg: msg.twist.angular.z > 0),
                (b's', 'stop', lambda msg: msg.twist.linear.x == msg.twist.angular.z == 0),
                (b'x', 'backward', lambda msg: msg.twist.linear.x < 0),
                (b'd', 'right', lambda msg: msg.twist.angular.z < 0),
                (b' ', 'stop', lambda msg: msg.twist.linear.x == msg.twist.angular.z == 0),
            ]
            for key, expected, predicate in keys:
                previous_count = len(messages)
                os.write(keyboard, key)
                lab.wait_for(lambda: len(messages) > previous_count and predicate(messages[-1]),
                             timeout=3, description=f'TurtleBot {expected}')
                assert messages[-1].header.stamp.sec > 0
                passed(f'TurtleBot: {key!r} sends stamped {expected} command')
            os.write(keyboard, b'\x03')
            process.wait(timeout=5)
            passed('TurtleBot: Ctrl+C after explicit stop exits teleop')
    finally:
        lab.node.destroy_subscription(subscription)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain-id', type=int, default=92)
    parser.add_argument('--output', type=Path, default=Path('/tmp/intro-cli-verification'))
    args = parser.parse_args()
    if not 0 <= args.domain_id <= 232:
        parser.error('--domain-id must be between 0 and 232')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    # Imports resolve beside this script, so running it from another working
    # directory still uses the current checkout's workshop helpers.
    from intro_ros import IntroLab
    from turtlesim.msg import Pose

    args.output.mkdir(parents=True, exist_ok=True)
    report = {'domain_id': args.domain_id,
              'exercise_directory': str(Path(__file__).resolve().parent),
              'checks': [], 'result': 'running'}

    def passed(name):
        report['checks'].append(name)
        print(f'PASS {name}', flush=True)

    lab = IntroLab('intro_cli_verification')
    try:
        lab.spin_for(1.0)
        existing_nodes = lab.node.get_node_names_and_namespaces()
        # CLI introspection can leave its discovery daemon alive after a run.
        # That passive node neither owns the robot nor sends motion commands.
        active_nodes = [(name, namespace) for name, namespace in existing_nodes
                        if not name.startswith('_ros2cli_daemon_')]
        assert active_nodes == [('intro_cli_verification', '/')], (
            f'Domain {args.domain_id} is in use; choose an unused --domain-id', existing_nodes)
        lab.start_turtlesim()
        assert lab.process is not None, 'Refusing to drive an unowned simulator'
        state = {'pose': None}
        lab.node.create_subscription(Pose, '/turtle1/pose', lambda msg: state.update(pose=msg), 10)
        lab.wait_for(lambda: state['pose'] is not None, description='turtlesim pose')
        check_turtle_keyboard(lab, state, passed)
        check_documented_cli(lab, state, passed)
        check_external_clients(lab, passed)
        check_turtlebot_keyboard(lab, passed)
        report['result'] = 'passed'
    except BaseException as error:
        report.update(result='failed', error=f'{type(error).__name__}: {error}')
        raise
    finally:
        try:
            lab.close()
        finally:
            (args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(f'{len(report["checks"])} checks passed; report: {args.output / "report.json"}', flush=True)


if __name__ == '__main__':
    main()
