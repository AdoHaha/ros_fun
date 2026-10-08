#!/usr/bin/env python3
"""Behavior Trees + Nav2 helper demo.

Run this after Gazebo and Nav2 are active:

    python3 trees_nav.py --auto-test

The same module is imported by the notebook so the exercise and the standalone
test use one implementation.
"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
import traceback
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Iterable

import py_trees
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ClearEntireCostmap
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.srv import SetParameters
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter


PATROL_WAYPOINTS = [
    (0.45, 1.91, 0.0),
    (2.53, 0.99, -1.57),
    (1.72, -1.37, 3.14),
    (-0.25, -1.15, 1.57),
]

# Face the direction of approach to keep the workshop demo short on slow hosts.
GOAL_POSE = (0.50, -1.20, -1.57)


class DemoMode(Enum):
    PATROL = "patrol"
    GOAL = "goal"
    STOPPED = "stopped"


@dataclass
class DemoState:
    mode: DemoMode = DemoMode.PATROL
    status: str = "Patrol wokół zamku"
    patrol_cycles: int = 0
    lock: threading.Lock = field(default_factory=threading.Lock)
    events: list[dict] = field(default_factory=list)

    def set_mode(self, mode: DemoMode, status: str, event: str | None = None, **data):
        with self.lock:
            self.mode = mode
            self.status = status
            if event:
                self.events.append(
                    {
                        "time": time.time(),
                        "event": event,
                        "mode": mode.value,
                        "status": status,
                        "data": data,
                    }
                )

    def snapshot(self):
        with self.lock:
            return self.mode, self.status, self.patrol_cycles

    def add_patrol_cycle(self):
        with self.lock:
            self.patrol_cycles += 1
            self.status = f"Cykl patrolu {self.patrol_cycles} zakończony; kontynuuję patrol"
            self.events.append(
                {
                    "time": time.time(),
                    "event": "patrol_cycle",
                    "mode": self.mode.value,
                    "status": self.status,
                    "data": {"cycles": self.patrol_cycles},
                }
            )

    def record(self, status: str, event: str, **data):
        """Update feedback without overwriting an intent submitted by the UI."""
        with self.lock:
            self.status = status
            self.events.append({"time": time.time(), "event": event,
                                "mode": self.mode.value, "status": status, "data": data})

    def transition(self, expected: DemoMode, mode: DemoMode, status: str, event: str):
        """Complete a request only if it has not been superseded by another intent."""
        with self.lock:
            if self.mode != expected:
                return False
            self.mode, self.status = mode, status
            self.events.append({"time": time.time(), "event": event,
                                "mode": mode.value, "status": status, "data": {}})
            return True

    def event_log(self) -> list[dict]:
        with self.lock:
            return list(self.events)


class DemoNode(Node):
    def __init__(self, name="castle_bt_demo_helper"):
        super().__init__(
            name,
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )


def bounded_service_call(node, client, request, description, timeout_sec=8.0):
    """Retry idempotent workshop services after missing replies, within a wall deadline."""
    deadline = time.monotonic() + timeout_sec
    reason = "service unavailable"
    while time.monotonic() < deadline:
        if not client.wait_for_service(timeout_sec=min(1.0, max(0.0, deadline - time.monotonic()))):
            continue
        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            node, future, timeout_sec=min(2.0, max(0.0, deadline - time.monotonic()))
        )
        if not future.done():
            reason = "service reply was not received"
            client.remove_pending_request(future)
            future.cancel()
            continue
        if future.exception() is not None:
            reason = str(future.exception())
            continue
        return future.result()
    raise TimeoutError(f"Timed out during {description}: {reason}. "
                       "Restart simulation/navigation with its notebook cell and retry.")


def configure_controller_frequency(node, desired_frequency=5.0):
    client = node.create_client(SetParameters, "/controller_server/set_parameters")
    request = SetParameters.Request()
    request.parameters = [Parameter("controller_frequency", value=float(desired_frequency)).to_parameter_msg()]
    try:
        response = bounded_service_call(node, client, request, "controller frequency configuration")
        if response is None or not response.results or not all(result.successful for result in response.results):
            reasons = [result.reason for result in response.results] if response else ["empty reply"]
            raise RuntimeError(f"Controller frequency configuration failed: {reasons}")
    finally:
        node.destroy_client(client)


class WorkshopNavigator(BasicNavigator):
    """BasicNavigator with a bounded startup, including retries for lost DDS replies."""

    def waitUntilNav2Active(self, navigator="bt_navigator", localizer="amcl", timeout_sec=60.0):
        deadline = time.monotonic() + timeout_sec
        if localizer != "robot_localization":
            self._wait_for_active_node(localizer, deadline)
        if localizer == "amcl":
            self._wait_for_initial_pose(deadline)
        self._wait_for_active_node(navigator, deadline)
        self.info("Nav2 is ready for use!")

    def _await_action_reply(self, future, description, timeout_sec=2.0):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            raise TimeoutError(f"Nav2 timed out waiting for {description}. "
                               "Restart navigation from its notebook cell.")
        if future.exception() is not None:
            raise RuntimeError(f"Nav2 {description} failed: {future.exception()}")
        return future.result()

    @staticmethod
    def _cancel_late_goal(future):
        # A goal accepted after our deadline must not begin an unowned navigation.
        if not future.cancelled() and future.exception() is None:
            handle = future.result()
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()

    def goToPose(self, pose, behavior_tree=""):
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if self.nav_to_pose_client.wait_for_server(
                timeout_sec=min(1.0, max(0.0, deadline - time.monotonic()))
            ):
                break
        else:
            raise TimeoutError("Nav2 navigate_to_pose server unavailable; restart navigation in the notebook.")
        goal = NavigateToPose.Goal()
        goal.pose, goal.behavior_tree = pose, behavior_tree
        self.feedback = None
        future = self.nav_to_pose_client.send_goal_async(goal, self._feedbackCallback)
        try:
            handle = self._await_action_reply(future, "goal acknowledgement")
        except TimeoutError:
            future.add_done_callback(self._cancel_late_goal)
            raise
        self.goal_handle = handle
        if handle is None or not handle.accepted:
            return False
        self.result_future = handle.get_result_async()
        return True

    def cancelTask(self):
        if self.result_future and self.goal_handle:
            self._await_action_reply(self.goal_handle.cancel_goal_async(), "cancellation acknowledgement")

    def clearAllCostmaps(self):
        # Both clears share one deadline, including when called during a tree tick.
        deadline = time.monotonic() + 8.0
        for client, description in [(self.clear_costmap_local_srv, "local costmap clearing"),
                                    (self.clear_costmap_global_srv, "global costmap clearing")]:
            bounded_service_call(self, client, ClearEntireCostmap.Request(), description,
                                 timeout_sec=max(0.0, deadline - time.monotonic()))

    def clearLocalCostmap(self):
        bounded_service_call(self, self.clear_costmap_local_srv, ClearEntireCostmap.Request(),
                             "local costmap clearing")

    def clearGlobalCostmap(self):
        bounded_service_call(self, self.clear_costmap_global_srv, ClearEntireCostmap.Request(),
                             "global costmap clearing")

    def _wait_for_active_node(self, node_name, deadline):
        service = f"{node_name}/get_state"
        client = self.create_client(GetState, service)
        reason = "service unavailable"
        try:
            while time.monotonic() < deadline:
                remaining = deadline - time.monotonic()
                if not client.wait_for_service(timeout_sec=min(1.0, remaining)):
                    continue
                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(
                    self, future, timeout_sec=min(2.0, max(0.0, deadline - time.monotonic()))
                )
                if not future.done():
                    reason = "no lifecycle response; DDS reply may have been lost"
                    # Discard this request and try again instead of waiting forever.
                    client.remove_pending_request(future)
                    future.cancel()
                    continue
                if future.exception() is not None:
                    reason = f"lifecycle request failed: {future.exception()}"
                    continue
                response = future.result()
                if response is not None and response.current_state.label == "active":
                    return
                reason = f"node state is {response.current_state.label if response else 'unknown'}"
                time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))
        finally:
            self.destroy_client(client)
        raise TimeoutError(f"Nav2 startup timed out waiting for {service}: {reason}. "
                           "Run the simulation restart cell, then retry navigation setup.")

    def _wait_for_initial_pose(self, deadline):
        while not self.initial_pose_received and time.monotonic() < deadline:
            self._setInitialPose()
            rclpy.spin_once(self, timeout_sec=min(1.0, max(0.0, deadline - time.monotonic())))
        if not self.initial_pose_received:
            raise TimeoutError("Nav2 startup timed out waiting for AMCL pose. "
                               "Check the simulation clock and restart navigation from its notebook cell.")


@dataclass
class DemoContext:
    node: Node
    navigator: BasicNavigator
    state: DemoState
    waypoints: list[tuple[float, float, float]]
    goal: tuple[float, float, float]
    cmd_vel_publisher: object | None = None
    navigation_owner: object | None = field(default=None, init=False)
    patrol_waypoint_index: int = field(default=0, init=False)
    cancellation_pending: bool = field(default=False, init=False)

    def cancel_navigation(self, owner=None):
        """Only the current owner may cancel the shared BasicNavigator task."""
        if self.navigation_owner is None:
            return
        if owner is not None and self.navigation_owner is not owner:
            return
        if not self.cancellation_pending:
            # cancelTask waits for acknowledgement, not the action's terminal result.
            self.navigator.cancelTask()
            self.cancellation_pending = True
        previous = self.navigation_owner
        self.navigation_owner = None
        self.state.record("Poprzednie zadanie anulowane", "navigation_canceled", owner=previous.name)

    def claim_navigation(self, owner):
        # Selector ticks a new high-priority child BEFORE invalidating the old one.
        # Transfer ownership here so its later terminate(INVALID) cannot cancel us.
        self.cancel_navigation()
        self.navigation_owner = owner

    def navigation_ready(self):
        """Poll cancellation completion before reusing the navigator action client."""
        if self.cancellation_pending:
            if not self.navigator.isTaskComplete():
                return False
            self.cancellation_pending = False
        return True

    def release_navigation(self, owner):
        if self.navigation_owner is owner:
            self.navigation_owner = None

    def shutdown(self):
        self.cancel_navigation()
        self.publish_stop()

    def yaw_to_quaternion(self, yaw: float) -> tuple[float, float]:
        return math.sin(yaw / 2.0), math.cos(yaw / 2.0)

    def make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.z, pose.pose.orientation.w = self.yaw_to_quaternion(float(yaw))
        return pose

    def goal_pose(self) -> PoseStamped:
        return self.make_pose(*self.goal)

    def request_goal(self) -> str:
        self.state.set_mode(DemoMode.GOAL, "Kliknięto: jedź do celu", "button_goal")
        return status_text(self.state)

    def request_stop(self) -> str:
        self.state.set_mode(DemoMode.STOPPED, "Kliknięto: stop", "button_stop")
        return status_text(self.state)

    def request_patrol(self) -> str:
        self.state.set_mode(
            DemoMode.PATROL,
            "Cel wyczyszczony; patrol jako fallback",
            "button_patrol",
        )
        return status_text(self.state)

    def publish_stop(self, repeats: int = 1):
        if self.cmd_vel_publisher is None:
            return
        msg = TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        for _ in range(repeats):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.cmd_vel_publisher.publish(msg)
            if repeats > 1:
                time.sleep(0.03)


def status_text(state: DemoState) -> str:
    mode, message, cycles = state.snapshot()
    return f"Tryb: {mode.value} | Cykle patrolu: {cycles} | Status: {message}"


class ModeIs(py_trees.behaviour.Behaviour):
    def __init__(self, context: DemoContext, name: str, expected_mode: DemoMode):
        super().__init__(name=name)
        self.context = context
        self.expected_mode = expected_mode

    def update(self):
        mode, status, _ = self.context.state.snapshot()
        self.feedback_message = status
        if mode == self.expected_mode:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class CancelNavigation(py_trees.behaviour.Behaviour):
    """Hold the stopped branch RUNNING until its guard becomes false."""

    def __init__(self, context: DemoContext, name="Anuluj nawigację"):
        super().__init__(name=name)
        self.context = context

    def initialise(self):
        self.context.cancel_navigation()
        self.context.publish_stop()
        self.context.state.record("Nawigacja anulowana; robot zatrzymany", "cancel_initialise")

    def update(self):
        self.feedback_message = "Robot zatrzymany"
        return py_trees.common.Status.RUNNING


class NavigateToGoal(py_trees.behaviour.Behaviour):
    """One Nav2 action: send on entry, poll while running, cancel on interruption.

    Guards and recovery belong to the surrounding tree, not this leaf.
    """

    def __init__(self, context: DemoContext, name="Jedź do celu"):
        super().__init__(name=name)
        self.context = context
        self.goal_sent = False
        self.goal_pending = False
        self.rejected = False

    def target(self):
        return self.context.goal

    def sent_event(self):
        return "goal_sent"

    def result_event(self):
        return "goal_result"

    def initialise(self):
        self.goal_sent = False
        self.goal_pending = True
        self.rejected = False
        self.context.claim_navigation(self)
        self.send_when_ready()

    def send_when_ready(self):
        if not self.context.navigation_ready():
            self.feedback_message = "Czekam na zakończenie anulowania poprzedniego zadania"
            return
        self.goal_pending = False
        x, y, yaw = self.target()
        try:
            accepted = self.context.navigator.goToPose(self.context.make_pose(x, y, yaw))
        except Exception:
            self.context.release_navigation(self)
            raise
        if accepted is False:
            self.rejected = True
            self.context.release_navigation(self)
            self.context.state.record("Nav2 odrzuciło cel", self.result_event(), result="REJECTED")
            return
        self.goal_sent = True
        self.context.state.record(
            f"{self.name}: ({x:.2f}, {y:.2f})", self.sent_event(),
            x=x, y=y, waypoint=self.context.patrol_waypoint_index + 1,
        )

    def update(self):
        if self.goal_pending:
            self.send_when_ready()
            if self.goal_pending:
                return py_trees.common.Status.RUNNING
        if self.rejected:
            self.feedback_message = "cel odrzucony"
            return py_trees.common.Status.FAILURE
        if not self.context.navigator.isTaskComplete():
            feedback = self.context.navigator.getFeedback()
            if feedback is not None:
                remaining = getattr(feedback, "estimated_time_remaining", None)
                if remaining is not None:
                    self.feedback_message = f"pozostało około {remaining.sec + remaining.nanosec / 1e9:.0f} s"
            return py_trees.common.Status.RUNNING
        result = self.context.navigator.getResult()
        self.goal_sent = False
        self.context.release_navigation(self)
        self.context.state.record(f"Wynik: {result}", self.result_event(), result=str(result))
        self.feedback_message = str(result)
        return (py_trees.common.Status.SUCCESS if result == TaskResult.SUCCEEDED
                else py_trees.common.Status.FAILURE)

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID:
            if self.goal_sent or self.goal_pending:
                self.context.cancel_navigation(owner=self)
            # Cancellation remains in the context when an unsent leaf is interrupted.
            self.goal_sent = False
            self.goal_pending = False
            self.rejected = False


class PatrolCastle(NavigateToGoal):
    """Navigate to one patrol waypoint; progression is a separate tree leaf."""

    def __init__(self, context: DemoContext, name="Punkt patrolu"):
        super().__init__(context, name)

    @property
    def waypoint_index(self):
        return self.context.patrol_waypoint_index

    def target(self):
        return self.context.waypoints[self.waypoint_index]

    def sent_event(self):
        return "patrol_goal_sent"

    def result_event(self):
        return "patrol_result"


class ClearCostmaps(py_trees.behaviour.Behaviour):
    def __init__(self, context, name="Wyczyść costmapy"):
        super().__init__(name)
        self.context = context

    def update(self):
        self.context.navigator.clearAllCostmaps()
        self.context.state.record("Costmapy wyczyszczone; jedna ponowna próba", "costmaps_cleared")
        return py_trees.common.Status.SUCCESS


class ChangeMode(py_trees.behaviour.Behaviour):
    def __init__(self, context, expected, target, name, event):
        super().__init__(name)
        self.context, self.expected, self.target, self.event = context, expected, target, event

    def update(self):
        changed = self.context.state.transition(self.expected, self.target, self.name, self.event)
        return py_trees.common.Status.SUCCESS if changed else py_trees.common.Status.FAILURE


class AdvancePatrol(py_trees.behaviour.Behaviour):
    def __init__(self, context, name="Następny punkt patrolu"):
        super().__init__(name)
        self.context = context

    def update(self):
        self.context.patrol_waypoint_index = (self.context.patrol_waypoint_index + 1) % len(self.context.waypoints)
        if self.context.patrol_waypoint_index == 0:
            self.context.state.add_patrol_cycle()
        return py_trees.common.Status.SUCCESS


def navigation_with_recovery(context, action_type, name):
    """Visible, bounded policy: attempt OR (clear costmaps THEN one retry)."""
    retry = py_trees.composites.Sequence(name=f"{name}: odzyskiwanie", memory=True)
    retry.add_children([ClearCostmaps(context), action_type(context, name=f"{name}: próba 2")])
    recovery = py_trees.composites.Selector(name=f"{name}: jedna ponowna próba", memory=True)
    recovery.add_children([action_type(context, name=f"{name}: próba 1"), retry])
    return recovery


def make_tree(context: DemoContext) -> py_trees.trees.BehaviourTree:
    if not context.waypoints:
        raise ValueError("Patrol requires at least one waypoint")
    root = py_trees.composites.Selector(name="Castle Demo", memory=False)

    stopped = py_trees.composites.Sequence(name="STOPPED?", memory=False)
    stopped.add_children([ModeIs(context, "tryb STOPPED?", DemoMode.STOPPED), CancelNavigation(context)])

    goal_work = py_trees.composites.Sequence(name="Wykonaj żądanie celu", memory=True)
    goal_work.add_children([
        navigation_with_recovery(context, NavigateToGoal, "Cel"),
        ChangeMode(context, DemoMode.GOAL, DemoMode.PATROL,
                   "Cel osiągnięty; wracam do patrolu", "goal_cleared_to_patrol"),
    ])
    goal_result = py_trees.composites.Selector(name="Cel lub bezpieczny stop", memory=True)
    goal_result.add_children([
        goal_work,
        ChangeMode(context, DemoMode.GOAL, DemoMode.STOPPED,
                   "Cel nieudany po dwóch próbach; stop", "goal_failed"),
    ])
    goal = py_trees.composites.Sequence(name="GOAL?", memory=False)
    goal.add_children([ModeIs(context, "tryb GOAL?", DemoMode.GOAL), goal_result])

    waypoint = py_trees.composites.Sequence(name="Jeden punkt patrolu", memory=True)
    waypoint.add_children([navigation_with_recovery(context, PatrolCastle, "Patrol"), AdvancePatrol(context)])
    patrol_result = py_trees.composites.Selector(name="Patrol lub bezpieczny stop", memory=True)
    patrol_result.add_children([
        py_trees.decorators.SuccessIsRunning(name="Powtarzaj punkty patrolu", child=waypoint),
        ChangeMode(context, DemoMode.PATROL, DemoMode.STOPPED,
                   "Patrol nieudany po dwóch próbach; stop", "patrol_failed"),
    ])
    patrol = py_trees.composites.Sequence(name="PATROL fallback", memory=False)
    patrol.add_children([ModeIs(context, "tryb PATROL?", DemoMode.PATROL), patrol_result])
    root.add_children([stopped, goal, patrol])
    return py_trees.trees.BehaviourTree(root)


class TreeRunner:
    def __init__(self, tree: py_trees.trees.BehaviourTree, period=0.5, on_tick=None):
        self.tree = tree
        self.period = period
        self.on_tick = on_tick
        self.running = False
        self.thread: threading.Thread | None = None
        self.last_error: BaseException | None = None

    def start(self):
        if self.running:
            print("Wątek drzewa już działa")
            return
        self.running = True
        self.last_error = None
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        print("Wątek drzewa uruchomiony")

    def stop(self, cancel=None):
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=20.0)
        if self.thread is not None and self.thread.is_alive():
            raise RuntimeError("Tree tick is still active; wait before destroying ROS nodes")
        self.tree.root.stop(py_trees.common.Status.INVALID)
        if cancel is not None:
            cancel()
        print("Wątek drzewa zatrzymany")

    def _run(self):
        while self.running:
            try:
                self.tree.tick()
                if self.on_tick is not None:
                    self.on_tick(self.tree)
                time.sleep(self.period)
            except Exception as exc:  # pragma: no cover - printed for notebook users
                self.last_error = exc
                self.running = False
                print("Błąd wątku drzewa:", repr(exc))
                traceback.print_exc()


class MotionMonitor(Node):
    def __init__(self):
        super().__init__("castle_bt_motion_monitor")
        self.odom_samples = 0
        self.cmd_samples = 0
        self.nonzero_cmd_samples = 0
        self.first_pose = None
        self.last_pose = None
        self.path_points = []
        self.path_length = 0.0
        self.last_cmd = None
        self.create_subscription(Odometry, "/odom", self._odom_callback, 10)
        self.create_subscription(TwistStamped, "/cmd_vel", self._cmd_stamped_callback, 10)

    def _odom_callback(self, msg):
        pose = msg.pose.pose.position
        point = (float(pose.x), float(pose.y))
        if self.first_pose is None:
            self.first_pose = point
        if self.last_pose is not None:
            self.path_length += math.hypot(point[0] - self.last_pose[0], point[1] - self.last_pose[1])
        self.last_pose = point
        if not self.path_points or math.hypot(point[0] - self.path_points[-1][0], point[1] - self.path_points[-1][1]) >= 0.02:
            self.path_points.append(point)
        self.odom_samples += 1

    def _cmd_stamped_callback(self, msg):
        self._record_cmd(msg.twist.linear.x, msg.twist.angular.z)

    def _record_cmd(self, linear_x, angular_z):
        self.cmd_samples += 1
        self.last_cmd = (float(linear_x), float(angular_z))
        if abs(linear_x) > 1e-4 or abs(angular_z) > 1e-4:
            self.nonzero_cmd_samples += 1

    def summary(self):
        return {
            "odom_samples": self.odom_samples,
            "cmd_samples": self.cmd_samples,
            "nonzero_cmd_samples": self.nonzero_cmd_samples,
            "first_pose": self.first_pose,
            "last_pose": self.last_pose,
            "path_length_m": round(self.path_length, 3),
            "path_points": len(self.path_points),
            "last_cmd": self.last_cmd,
        }


def draw_path_png(
    output_path: str | Path,
    path_points: list[tuple[float, float]],
    map_yaml="/home/ubuntu/turtlebot3_ws/src/jupyter_notebooks/map.yaml",
    waypoints=PATROL_WAYPOINTS,
    goal=GOAL_POSE,
    scale=4,
):
    from PIL import Image, ImageDraw

    metadata = {}
    for raw_line in Path(map_yaml).read_text().splitlines():
        if ":" not in raw_line:
            continue
        key, value = raw_line.split(":", 1)
        metadata[key.strip()] = value.strip()

    image_path = metadata["image"]
    resolution = float(metadata.get("resolution", "0.05"))
    origin_text = metadata.get("origin", "[0, 0, 0]").strip("[]")
    origin_x, origin_y, _ = [float(part.strip()) for part in origin_text.split(",")]

    image = Image.open(image_path).convert("RGB")
    draw = ImageDraw.Draw(image)
    width, height = image.size

    def to_pixel(point):
        x, y = point[:2]
        px = int(round((x - origin_x) / resolution))
        py = int(round(height - (y - origin_y) / resolution))
        return px, py

    for x, y, _ in waypoints:
        px, py = to_pixel((x, y))
        draw.ellipse((px - 4, py - 4, px + 4, py + 4), fill=(255, 165, 0), outline=(80, 50, 0))

    gx, gy, _ = goal
    gpx, gpy = to_pixel((gx, gy))
    draw.rectangle((gpx - 5, gpy - 5, gpx + 5, gpy + 5), fill=(128, 0, 180), outline=(40, 0, 60))

    if len(path_points) >= 2:
        pixels = [to_pixel(point) for point in path_points]
        draw.line(pixels, fill=(220, 20, 60), width=3)
        sx, sy = pixels[0]
        ex, ey = pixels[-1]
        draw.ellipse((sx - 5, sy - 5, sx + 5, sy + 5), fill=(0, 160, 0), outline=(0, 60, 0))
        draw.ellipse((ex - 5, ey - 5, ex + 5, ey + 5), fill=(0, 90, 220), outline=(0, 30, 80))

    if scale > 1:
        image = image.resize((width * int(scale), height * int(scale)), Image.Resampling.NEAREST)

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    image.save(output_path)
    return str(output_path)


def setup_navigation(
    helper_services=None,
    waypoints: Iterable[tuple[float, float, float]] | None = None,
    goal: tuple[float, float, float] = GOAL_POSE,
) -> tuple[DemoNode, DemoContext]:
    try:
        rclpy.init()
    except RuntimeError:
        pass

    demo_node = DemoNode()
    if helper_services is not None:
        try:
            configure_controller_frequency(demo_node)
            helper_services.publish_initial_pose(demo_node)
        except Exception:
            demo_node.destroy_node()
            raise

    navigator = WorkshopNavigator()
    # Goals and initial poses must use the same clock as Gazebo and Nav2.
    navigator.set_parameters([Parameter("use_sim_time", value=True)])
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = "map"
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.08
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)
    try:
        navigator.waitUntilNav2Active()
        navigator.clearAllCostmaps()
    except Exception:
        navigator.destroy_node()
        demo_node.destroy_node()
        raise

    context = DemoContext(
        node=demo_node,
        navigator=navigator,
        state=DemoState(),
        waypoints=list(waypoints or PATROL_WAYPOINTS),
        goal=goal,
    )
    context.cmd_vel_publisher = demo_node.create_publisher(TwistStamped, "/cmd_vel", 10)
    context.state.set_mode(DemoMode.PATROL, "Patrol wokół zamku", "tree_started")
    return demo_node, context


def run_auto_test(args) -> int:
    import helper_services

    demo_node, context = setup_navigation(helper_services)
    tree = make_tree(context)
    tree.setup(timeout=15)
    monitor = MotionMonitor()
    monitor_executor = SingleThreadedExecutor()
    monitor_executor.add_node(monitor)
    monitor_running = True

    def spin_monitor():
        while monitor_running:
            monitor_executor.spin_once(timeout_sec=0.05)

    monitor_thread = threading.Thread(target=spin_monitor, daemon=True)
    monitor_thread.start()
    start = time.time()
    goal_requested = False
    stop_requested = False
    fallback_seen = False

    try:
        while time.time() - start < args.duration:
            elapsed = time.time() - start
            tree.tick()

            events = context.state.event_log()
            fallback_seen = any(e["event"] == "goal_cleared_to_patrol" for e in events) and any(
                e["event"] == "patrol_goal_sent"
                and e["time"] > next(
                    ge["time"] for ge in events if ge["event"] == "goal_cleared_to_patrol"
                )
                for e in events
            )

            if not goal_requested and elapsed >= args.goal_after:
                print(context.request_goal())
                goal_requested = True

            if goal_requested and fallback_seen and not stop_requested and elapsed >= args.stop_after:
                print(context.request_stop())
                tree.tick()  # Consume the intent through the STOP branch before ending.
                stop_requested = context.navigation_owner is None
                break

            time.sleep(args.period)
    finally:
        tree.root.stop(py_trees.common.Status.INVALID)
        context.shutdown()
        context.publish_stop(repeats=10)
        time.sleep(0.5)
        monitor_running = False
        monitor_thread.join(timeout=2.0)
        monitor_executor.remove_node(monitor)
        monitor_executor.shutdown()
        context.navigator.destroy_node()
        demo_node.destroy_node()
        monitor.destroy_node()
        rclpy.try_shutdown()

    summary = {
        "events": context.state.event_log(),
        "motion": monitor.summary(),
        "fallback_patrol_after_goal": fallback_seen,
        "stop_sent": stop_requested,
    }
    if args.output:
        Path(args.output).write_text(json.dumps(summary, ensure_ascii=False, indent=2))
    if args.path_png:
        summary["path_png"] = draw_path_png(args.path_png, monitor.path_points, scale=args.path_scale)
    print(json.dumps(summary, ensure_ascii=False, indent=2))

    moving = summary["motion"]["path_length_m"] >= args.min_path
    commanded = summary["motion"]["nonzero_cmd_samples"] >= args.min_cmd_samples
    if fallback_seen and stop_requested and moving and commanded:
        return 0
    return 2


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--auto-test", action="store_true", help="run goal/patrol/stop scenario")
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--goal-after", type=float, default=18.0)
    parser.add_argument("--stop-after", type=float, default=80.0)
    parser.add_argument("--period", type=float, default=0.5)
    parser.add_argument("--min-path", type=float, default=2.0)
    parser.add_argument("--min-cmd-samples", type=int, default=20)
    parser.add_argument("--output", default="/tmp/castle_bt_auto_test.json")
    parser.add_argument("--path-png", default="", help="draw odom path over map image")
    parser.add_argument("--path-scale", type=int, default=4, help="integer scale for --path-png")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.auto_test:
        raise SystemExit(run_auto_test(args))
    print("Uruchom z --auto-test albo importuj moduł w notebooku.")


if __name__ == "__main__":
    main()
