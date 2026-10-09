"""Small, bounded ROS utilities for exercises 1–5.

The notebooks keep publishers, subscribers, timers and service callbacks visible.
This module owns only waiting, simulator startup and cleanup; no background spinner.
"""
import math
import os
from pathlib import Path
import subprocess
import tempfile
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from turtlesim.srv import TeleportAbsolute


def front_distance(scan, half_angle=math.radians(20)):
    """Nearest valid reading in the forward cone, or None for unknown space.

    Use actual LaserScan angles/limits, not an assumed length or ranges[0].
    Infinity means no return; NaN/zero/out-of-range readings are not obstacles.
    An entirely invalid cone must not be interpreted as clear space.
    """
    readings = []
    for index, distance in enumerate(scan.ranges):
        angle = scan.angle_min + index * scan.angle_increment
        angle = math.atan2(math.sin(angle), math.cos(angle))
        if abs(angle) <= half_angle and math.isfinite(distance):
            if scan.range_min <= distance <= scan.range_max and distance > 0:
                readings.append(distance)
    return min(readings) if readings else None


def notebook_lab(name, notebook_state):
    """Create a fresh lab after releasing this notebook's previous resources.

    Pass globals() from the setup cell. Keeping rerun housekeeping here lets
    beginners focus on publishers, callbacks and requests in the lesson cells.
    """
    panel = notebook_state.pop('panel', None)
    if panel is not None:
        for button, callback in notebook_state.get('przyciski', []):
            button.on_click(callback, remove=True)
            button.close()
        panel.close()
    status = notebook_state.pop('status', None)
    if status is not None:
        status.close()
    previous_lab = notebook_state.get('lab')
    if previous_lab is not None:
        previous_lab.close()
    # ROS entities belonged to the old node; don't reuse destroyed handles.
    for key in ('subscription', 'gra_subscription', 'laser_subscription',
                'pose_subscription', 'server', 'ruch', 'timer'):
        notebook_state.pop(key, None)
    return IntroLab(name)


class IntroLab:
    """One notebook's ROS context, executor and optionally owned turtlesim process."""

    def __init__(self, name):
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = Node(name, context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.process = None
        self.log_path = None
        self._motion_publishers = []
        self._closed = False

    def spin_for(self, seconds):
        """Process callbacks for a finite amount of wall time."""
        if not math.isfinite(seconds) or seconds < 0:
            raise ValueError("Czas musi być skończony i nieujemny.")
        deadline = time.monotonic() + seconds
        while self.context.ok() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=min(0.05, max(0, deadline - time.monotonic())))

    def wait_for(self, predicate, timeout=5.0, description="dane ROS"):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() >= deadline:
                raise TimeoutError(f"Brak: {description}. Sprawdź, czy symulator działa i czy nazwy interfejsów są poprawne.")
            self.spin_for(0.05)

    def wait_for_subscriber(self, publisher, timeout=5.0):
        self.wait_for(lambda: publisher.get_subscription_count() > 0,
                      timeout, f"odbiorca topicu {publisher.topic_name}")

    def call(self, service_type, name, request, timeout=5.0):
        """Wait for discovery AND reply with a single wall-time deadline."""
        client = self.node.create_client(service_type, name)
        deadline = time.monotonic() + timeout
        future = None
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                raise TimeoutError(f"Brak serwisu {name}. Uruchom odpowiedni symulator lub serwer.")
            future = client.call_async(request)
            self.wait_for(future.done, max(0, deadline - time.monotonic()), f"odpowiedź {name}")
            return future.result()  # propagate transport/callback errors
        finally:
            if future is not None and not future.done():
                client.remove_pending_request(future)
                future.cancel()
            self.node.destroy_client(client)

    def start_turtlesim(self):
        """Reuse an existing simulator; stop only a process started by this lab."""
        client = self.node.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        try:
            # A fresh DDS participant may need several seconds to discover an
            # existing simulator, especially while Gazebo loads on a slow host.
            if client.wait_for_service(timeout_sec=5.0):
                print('Używam uruchomionego turtlesim. Zatrzymaj pozostałe programy sterujące żółwiem.')
                return
        finally:
            self.node.destroy_client(client)
        env = os.environ.copy()
        env.setdefault('DISPLAY', ':1.0')
        with tempfile.NamedTemporaryFile(prefix='ros_fun_turtlesim_', suffix='.log', delete=False) as log:
            self.log_path = Path(log.name)
            self.process = subprocess.Popen(['ros2', 'run', 'turtlesim', 'turtlesim_node'],
                                            env=env, stdout=log, stderr=subprocess.STDOUT,
                                            start_new_session=True)
        client = self.node.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        try:
            if not client.wait_for_service(timeout_sec=15.0):
                self._stop_process()
                raise RuntimeError(f'Turtlesim nie wystartował. Log: {self.log_path}')
        finally:
            self.node.destroy_client(client)
        print('Turtlesim gotowy. Otwórz pulpit http://localhost:6080 (Connect).')

    def watch_motion(self, publisher):
        """Register a velocity publisher for a final zero command on close."""
        if publisher not in self._motion_publishers:
            self._motion_publishers.append(publisher)

    def velocity(self, publisher, linear=0.0, angular=0.0):
        msg = publisher.msg_type()
        if isinstance(msg, TwistStamped):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            twist = msg.twist
        elif isinstance(msg, Twist):
            twist = msg
        else:
            raise TypeError('Publisher musi wysyłać Twist albo TwistStamped.')
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        publisher.publish(msg)

    def drive(self, publisher, linear=0.0, angular=0.0, seconds=0.5):
        """Send at 10 Hz for a bounded interval; zero even on an exception."""
        if not math.isfinite(seconds) or not 0 <= seconds <= 10:
            raise ValueError('Jeden ruch powinien trwać od 0 do 10 sekund.')
        self.watch_motion(publisher)
        self.wait_for_subscriber(publisher)
        deadline = time.monotonic() + seconds
        try:
            while time.monotonic() < deadline:
                self.velocity(publisher, linear, angular)
                self.spin_for(min(0.1, max(0, deadline - time.monotonic())))
        finally:
            self.velocity(publisher)
            self.spin_for(0.1)

    def _stop_process(self):
        if self.process is not None:
            import signal
            try:
                os.killpg(self.process.pid, signal.SIGTERM)
                self.process.wait(timeout=3)
            except ProcessLookupError:
                pass
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait(timeout=3)
            self.process = None

    def close(self):
        if self._closed:
            return
        try:
            for publisher in self._motion_publishers:
                self.velocity(publisher)
            self.spin_for(0.15)
        finally:
            self._stop_process()
            self.executor.remove_node(self.node)
            self.node.destroy_node()
            self.executor.shutdown()
            self.context.shutdown()
            self._closed = True
