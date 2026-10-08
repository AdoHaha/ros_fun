"""Use the exercise 10 tree unchanged with real ROS topics and action clients."""

import time

import py_trees
import py_trees_ros
import rclpy
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Empty, String
from py_trees_ros_interfaces.action import Rotate

from bt_workshop import World, Record, Led


class CancellableActionClient(py_trees_ros.actions.ActionClient):
    """Also cancel goals whose acceptance arrives after tree interruption.

    py_trees_ros 2.4 cancels an already accepted goal. This adapter covers the
    outstanding send-goal future and ignores results from a previous entry.
    """
    def initialise(self):
        self.cancel_before_accept = False
        super().initialise()

    def terminate(self, new_status):
        if self.status == py_trees.common.Status.RUNNING and new_status == py_trees.common.Status.INVALID:
            self.cancel_before_accept = True
        super().terminate(new_status)

    def goal_response_callback(self, future):
        handle = future.result()
        if future is not self.send_goal_future or self.cancel_before_accept:
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
            return
        super().goal_response_callback(future)

    def get_result_callback(self, future):
        if future is self.get_result_future:
            super().get_result_callback(future)


class RosLab:
    def __init__(self, builder):
        self.owns_context = not rclpy.ok()
        if self.owns_context:
            rclpy.init()
        self.node = rclpy.create_node('workshop_mission')
        self.closed = False
        self.world = World(battery_low=True)  # wait for the first battery reading
        # One writer publishes the selected output after tree invalidation has
        # finished, so cleanup of the blue branch cannot erase a new red alarm.
        self.led_publisher = self.node.create_publisher(String, '/led_strip/command', 10)
        self.subscriptions = [
            self.node.create_subscription(BatteryState, '/battery/state', self.battery, 10),
            self.node.create_subscription(Empty, '/dashboard/scan', self.scan, 10),
            self.node.create_subscription(Empty, '/dashboard/cancel', self.cancel, 10),
        ]
        def action(name):
            return CancellableActionClient(
                name=name, action_type=Rotate, action_name='/rotate',
                action_goal=Rotate.Goal(), wait_for_server_timeout_sec=20.0,
            )
        led = Led('Blue', self.world, 'blue')
        root = builder(self.world, action('Scan'), led, Record('Repair', self.world), action('Retry'))
        self.tree = py_trees.trees.BehaviourTree(root)
        try:
            self.tree.setup(timeout=45, node=self.node)
        except Exception:
            self.close()
            raise

    def battery(self, msg):
        # The workshop mock publishes percentages in [0, 100].
        # A hardware adapter must normalise its own BatteryState convention.
        self.world.battery_low = msg.percentage < 30.0

    def scan(self, msg):
        self.world.scan_requested = True

    def cancel(self, msg):
        self.world.cancel_requested = True

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0.05)
        self.tree.tick()
        self.led_publisher.publish(String(data=self.world.led or ''))

    def run_until(self, predicate, timeout=15):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.step()
            if predicate():
                return
            time.sleep(0.05)
        raise TimeoutError('Nie osiągnięto oczekiwanego stanu drzewa: ' + str(self.world.events))

    def set_battery(self, percentage):
        client = AsyncParameterClient(self.node, '/battery')
        if not client.wait_for_services(timeout_sec=5):
            raise TimeoutError('Mock /battery nie udostępnia usług parametrów')
        future = client.set_parameters([Parameter('charging_percentage', value=float(percentage))])
        self.run_until(future.done, timeout=5)
        response = future.result()
        if response is None or not all(result.successful for result in response.results):
            raise RuntimeError('Mock odrzucił zmianę baterii: ' + str(response))

    def close(self):
        if self.closed:
            return
        self.closed = True
        # Allow asynchronous cancellation to reach the action server before
        # destroying the action clients and their node.
        self.tree.root.stop(py_trees.common.Status.INVALID)
        self.led_publisher.publish(String(data=''))
        deadline = time.monotonic() + 0.5
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        for behaviour in self.tree.root.iterate():
            if hasattr(behaviour, 'action_client') and behaviour.action_client is None:
                continue
            behaviour.shutdown()
        self.node.destroy_node()
        if self.owns_context:
            rclpy.try_shutdown()
