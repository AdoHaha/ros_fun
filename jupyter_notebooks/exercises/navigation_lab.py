"""Bounded, owned Nav2 setup for exercise 6; navigation requests stay visible.

No background executor and no broad process cleanup. Each kernel owns only the
Gazebo/Nav2/RViz process groups it starts, and refuses an existing navigation world.
"""
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException

from trees_nav import WorkshopNavigator, configure_controller_frequency
from nav2_simple_commander.robot_navigator import TaskResult


def _validate_duration(seconds):
    if not math.isfinite(seconds) or seconds < 0:
        raise ValueError('Czas musi być skończoną, nieujemną liczbą.')


def goal_pose(navigator, x, y, yaw):
    """Map coordinates in metres; yaw in radians, converted to a unit quaternion."""
    if not all(math.isfinite(value) for value in (x, y, yaw)):
        raise ValueError('Pozycja i kąt muszą być skończonymi liczbami.')
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x, pose.pose.position.y = float(x), float(y)
    pose.pose.orientation.z = math.sin(yaw / 2)
    pose.pose.orientation.w = math.cos(yaw / 2)
    return pose


def stop_owned_process(process):
    # A launcher's shell may exit before its grandchildren. Keep signalling the
    # owned group until it disappears; a parent exit alone is not enough.
    for sig, timeout in ((signal.SIGINT, 5), (signal.SIGTERM, 3), (signal.SIGKILL, 3)):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            break
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            continue


class NavigationLab:
    def __init__(self, show_rviz=False, output='/tmp/courier-navigation'):
        self.processes = []
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=True)
        self.navigator = None
        self.tf_listener = None
        self.active = False
        self.closed = False
        self.last_result = None
        self.odom = None
        self.localized_pose = None
        self.odom_samples = 0
        self.pose_samples = 0
        self.path_length = 0.0
        self.last_cmd = None
        self.nonzero_commands = 0
        self.owns_context = not rclpy.ok()
        if self.owns_context:
            rclpy.init()
        try:
            self.navigator = WorkshopNavigator(node_name='courier_navigator')
            self.navigator.set_parameters([Parameter('use_sim_time', value=True)])
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self.navigator, spin_thread=False)
            self.navigator.create_subscription(Odometry, '/odom', self._receive_odom, 10)
            self.navigator.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                                self._receive_pose, 10)
            self.navigator.create_subscription(TwistStamped, '/cmd_vel', self._receive_command, 10)
            self.stop_publisher = self.navigator.create_publisher(TwistStamped, '/cmd_vel', 10)
            self.spin_for(5)
            nodes = self.navigator.get_node_names_and_namespaces()
            occupied = any(name in ('ros_gz_bridge', 'robot_state_publisher', 'amcl',
                                    'bt_navigator', 'controller_server') for name, _ in nodes)
            occupied |= sum(name == 'courier_navigator' for name, _ in nodes) > 1
            occupied |= self.navigator.count_publishers('/clock') > 0
            if occupied:
                raise RuntimeError('Gazebo/Nav2 już działa w tym ROS_DOMAIN_ID. Zakończ wcześniejszy pokaz '
                                   'jego komórką sprzątania lub Ctrl+C w jego terminalu. '
                                   'Nie uruchomię drugiego świata ani nie zamknę cudzego procesu.')
            self._launch('gazebo', ['ros2', 'launch', 'turtlebot3_gazebo',
                                    'turtlebot3_world.launch.py'])
            self.wait_for(lambda: self.navigator.get_clock().now().nanoseconds > 0,
                          timeout=90, description='zegar Gazebo /clock')
            params = Path(get_package_share_directory('turtlebot3_navigation2')) / 'param/waffle_pi.yaml'
            map_path = Path(__file__).resolve().parent.parent / 'map.yaml'
            self._launch('nav2', ['ros2', 'launch', 'nav2_bringup', 'bringup_launch.py',
                                  'autostart:=True', 'use_sim_time:=True',
                                  f'map:={map_path}', f'params_file:={params}'])
            self.navigator.setInitialPose(goal_pose(self.navigator, 0.08, 0.0, 0.0))
            self.navigator.waitUntilNav2Active(timeout_sec=120)
            configure_controller_frequency(self.navigator)
            self.wait_for(lambda: self.odom is not None, description='odometria /odom')
            if show_rviz:
                config = Path(get_package_share_directory('turtlebot3_navigation2')) / 'rviz/tb3_navigation2.rviz'
                self._launch('rviz', ['ros2', 'run', 'rviz2', 'rviz2', '-d', str(config),
                                      '--ros-args', '-p', 'use_sim_time:=true'])
            print('Gazebo, lokalizacja AMCL i Nav2 są gotowe. Cele podajemy w układzie map, w metrach.')
        except BaseException:
            self.close()
            raise

    def _launch(self, name, command):
        log_path = self.output / f'{name}.log'
        env = os.environ.copy()
        env.setdefault('DISPLAY', ':1.0')
        # Gazebo Transport has its own isolation in addition to the ROS domain.
        env['GZ_PARTITION'] = f'courier_navigation_{os.getpid()}'
        with log_path.open('w') as log:
            process = subprocess.Popen(command, env=env, stdout=log,
                                       stderr=subprocess.STDOUT, start_new_session=True)
        self.processes.append((name, process, log_path))

    def _check_processes(self):
        for name, process, log in self.processes:
            if process.poll() is not None:
                raise RuntimeError(f'{name} zakończył się przed czasem. Przeczytaj log: {log}')

    def _receive_odom(self, message):
        self.odom_samples += 1
        if self.odom is not None:
            previous = self.odom.pose.pose.position
            current = message.pose.pose.position
            self.path_length += math.hypot(current.x - previous.x, current.y - previous.y)
        self.odom = message

    def _receive_command(self, message):
        self.last_cmd = (message.twist.linear.x, message.twist.angular.z)
        if abs(self.last_cmd[0]) > 0.001 or abs(self.last_cmd[1]) > 0.001:
            self.nonzero_commands += 1

    def _receive_pose(self, message):
        self.pose_samples += 1
        self.localized_pose = message.pose.pose

    def map_position(self):
        """Latest TF position; AMCL publishes only after sufficient robot movement."""
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
        except TransformException:
            return None
        point = transform.transform.translation
        return point.x, point.y

    def spin_for(self, seconds):
        _validate_duration(seconds)
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.navigator, timeout_sec=min(0.05, max(0, deadline-time.monotonic())))

    def wait_for(self, predicate, timeout=15, description='dane ROS'):
        _validate_duration(timeout)
        deadline = time.monotonic() + timeout
        while not predicate():
            self._check_processes()
            if time.monotonic() >= deadline:
                raise TimeoutError(f'Brak: {description}. Logi: {self.output}')
            self.spin_for(0.05)

    def begin(self, pose):
        """Send once; acknowledgement of acceptance is not arrival."""
        if self.active:
            raise RuntimeError('Najpierw zakończ lub anuluj poprzedni cel i odbierz jego wynik.')
        self.last_result = None
        accepted = self.navigator.goToPose(pose)
        self.active = bool(accepted)
        if not accepted:
            raise RuntimeError('Nav2 odrzucił cel. To nie jest sukces dostawy.')
        return accepted

    def wait_result(self, timeout=300, feedback=None):
        _validate_duration(timeout)
        deadline = time.monotonic() + timeout
        next_feedback = 0
        try:
            while self.active:
                self._check_processes()
                if self.navigator.isTaskComplete():
                    self.last_result = self.navigator.getResult()
                    self.active = False
                    break
                if time.monotonic() >= deadline:
                    raise TimeoutError('Przejazd przekroczył limit czasu komputera. Anuluję cel.')
                value = self.navigator.getFeedback()
                if feedback is not None and value is not None and time.monotonic() >= next_feedback:
                    feedback(value)
                    next_feedback = time.monotonic() + 5
            return self.last_result
        except BaseException:
            self.cancel()
            raise

    def cancel(self, timeout=30):
        """Wait for the terminal result, not just cancel request acknowledgement."""
        _validate_duration(timeout)
        if not self.active:
            return self.last_result
        self.navigator.cancelTask()
        deadline = time.monotonic() + timeout
        while not self.navigator.isTaskComplete():
            if time.monotonic() >= deadline:
                raise TimeoutError('Serwer nie potwierdził końca anulowanej akcji. Nie wysyłaj kolejnego celu.')
        self.last_result = self.navigator.getResult()
        self.active = False
        return self.last_result

    def close(self):
        if self.closed:
            return
        try:
            if self.navigator is not None:
                try:
                    self.cancel(timeout=10)
                    # Owned Nav2 remains alive during settling; it must cease
                    # publishing before another controller starts.
                    if self.processes:
                        for _ in range(3):
                            msg = TwistStamped()
                            msg.header.stamp = self.navigator.get_clock().now().to_msg()
                            self.stop_publisher.publish(msg)
                            self.spin_for(0.1)
                finally:
                    try:
                        if self.tf_listener is not None:
                            self.tf_listener.unregister()
                    finally:
                        self.navigator.destroy_node()
        finally:
            try:
                for _, process, _ in reversed(self.processes):
                    stop_owned_process(process)
            finally:
                if self.owns_context:
                    rclpy.try_shutdown()
                self.closed = True
