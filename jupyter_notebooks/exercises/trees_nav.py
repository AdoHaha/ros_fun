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

GOAL_POSE = (0.50, -1.20, 0.0)


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

    def event_log(self) -> list[dict]:
        with self.lock:
            return list(self.events)


class DemoNode(Node):
    def __init__(self, name="castle_bt_demo_helper"):
        super().__init__(
            name,
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )


@dataclass
class DemoContext:
    node: Node
    navigator: BasicNavigator
    state: DemoState
    waypoints: list[tuple[float, float, float]]
    goal: tuple[float, float, float]
    cmd_vel_publisher: object | None = None

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
        self.navigator.cancelTask()
        self.publish_stop()
        return status_text(self.state)

    def request_patrol(self) -> str:
        self.navigator.cancelTask()
        self.publish_stop()
        self.state.set_mode(
            DemoMode.PATROL,
            "Cel wyczyszczony; patrol jako fallback",
            "button_patrol",
        )
        return status_text(self.state)

    def publish_stop(self, repeats: int = 5):
        if self.cmd_vel_publisher is None:
            return
        msg = TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        for _ in range(repeats):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.cmd_vel_publisher.publish(msg)
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
    def __init__(self, context: DemoContext, name="Anuluj nawigację"):
        super().__init__(name=name)
        self.context = context

    def initialise(self):
        self.context.navigator.cancelTask()
        self.context.publish_stop()
        self.context.state.set_mode(
            DemoMode.STOPPED,
            "Nawigacja anulowana; robot zatrzymany",
            "cancel_initialise",
        )

    def update(self):
        self.feedback_message = "Zażądano stopu"
        return py_trees.common.Status.RUNNING


class NavigateToGoal(py_trees.behaviour.Behaviour):
    def __init__(self, context: DemoContext, name="Jedź do celu"):
        super().__init__(name=name)
        self.context = context
        self.goal_sent = False

    def initialise(self):
        self.context.navigator.cancelTask()
        time.sleep(0.2)
        self.context.navigator.goToPose(self.context.goal_pose())
        self.goal_sent = True
        x, y, _ = self.context.goal
        self.context.state.set_mode(
            DemoMode.GOAL,
            f"Jadę do celu ({x:.2f}, {y:.2f})",
            "goal_sent",
            x=x,
            y=y,
        )

    def update(self):
        mode, _, _ = self.context.state.snapshot()
        if mode == DemoMode.STOPPED:
            self.context.navigator.cancelTask()
            self.context.publish_stop()
            return py_trees.common.Status.FAILURE

        if not self.goal_sent or not self.context.navigator.isTaskComplete():
            feedback = self.context.navigator.getFeedback()
            if feedback is not None:
                remaining = getattr(feedback, "estimated_time_remaining", None)
                if remaining is not None:
                    seconds = remaining.sec + remaining.nanosec / 1e9
                    self.feedback_message = f"cel aktywny, około {seconds:.0f} s do końca"
            return py_trees.common.Status.RUNNING

        result = self.context.navigator.getResult()
        self.goal_sent = False
        self.context.state.set_mode(
            self.context.state.snapshot()[0],
            f"Wynik celu: {result}",
            "goal_result",
            result=str(result),
        )
        if result == TaskResult.SUCCEEDED:
            self.context.state.set_mode(
                DemoMode.PATROL,
                "Cel osiągnięty; wracam do patrolu",
                "goal_cleared_to_patrol",
            )
            self.feedback_message = "cel osiągnięty, patrol jako fallback"
            return py_trees.common.Status.SUCCESS
        if result == TaskResult.CANCELED:
            self.context.state.set_mode(
                DemoMode.PATROL,
                "Cel anulowany; wracam do patrolu",
                "goal_canceled_to_patrol",
            )
            self.feedback_message = "cel anulowany, wracam do patrolu"
            return py_trees.common.Status.FAILURE
        self.context.state.set_mode(
            DemoMode.STOPPED,
            "Cel nieudany; robot zatrzymany",
            "goal_failed",
            result=str(result),
        )
        self.feedback_message = "cel nieudany"
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        mode, _, _ = self.context.state.snapshot()
        if new_status == py_trees.common.Status.INVALID and self.goal_sent and mode == DemoMode.GOAL:
            self.context.navigator.cancelTask()
            self.goal_sent = False


class PatrolCastle(py_trees.behaviour.Behaviour):
    def __init__(self, context: DemoContext, name="Patrol zamku"):
        super().__init__(name=name)
        self.context = context
        self.goal_sent = False
        self.waypoint_index = 0

    def send_next_waypoint(self):
        x, y, yaw = self.context.waypoints[self.waypoint_index]
        self.context.navigator.goToPose(self.context.make_pose(x, y, yaw))
        self.goal_sent = True
        self.context.state.set_mode(
            DemoMode.PATROL,
            f"Patrol: punkt {self.waypoint_index + 1}/{len(self.context.waypoints)}",
            "patrol_goal_sent",
            waypoint=self.waypoint_index + 1,
            x=x,
            y=y,
        )

    def initialise(self):
        if not self.goal_sent:
            self.send_next_waypoint()

    def update(self):
        mode, _, cycles = self.context.state.snapshot()
        if mode != DemoMode.PATROL:
            self.context.navigator.cancelTask()
            self.goal_sent = False
            return py_trees.common.Status.FAILURE

        if not self.goal_sent or not self.context.navigator.isTaskComplete():
            feedback = self.context.navigator.getFeedback()
            if feedback is not None:
                remaining = getattr(feedback, "estimated_time_remaining", None)
                if remaining is not None:
                    seconds = remaining.sec + remaining.nanosec / 1e9
                    self.feedback_message = (
                        f"cykl {cycles}, punkt {self.waypoint_index + 1}, "
                        f"około {seconds:.0f} s do końca"
                    )
            return py_trees.common.Status.RUNNING

        result = self.context.navigator.getResult()
        self.goal_sent = False
        if result == TaskResult.SUCCEEDED:
            self.waypoint_index = (self.waypoint_index + 1) % len(self.context.waypoints)
            if self.waypoint_index == 0:
                self.context.state.add_patrol_cycle()
            self.send_next_waypoint()
            return py_trees.common.Status.RUNNING
        if result == TaskResult.CANCELED:
            self.feedback_message = "patrol anulowany"
            return py_trees.common.Status.FAILURE
        self.feedback_message = "patrol nieudany; czyszczę costmapy i spróbuję ponownie"
        self.context.navigator.clearAllCostmaps()
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        mode, _, _ = self.context.state.snapshot()
        if new_status == py_trees.common.Status.INVALID and self.goal_sent and mode == DemoMode.PATROL:
            self.context.navigator.cancelTask()
            self.goal_sent = False


def make_tree(context: DemoContext) -> py_trees.trees.BehaviourTree:
    root = py_trees.composites.Selector(name="Castle Demo", memory=False)

    stopped = py_trees.composites.Sequence(name="STOPPED?", memory=True)
    stopped.add_children(
        [ModeIs(context, "tryb STOPPED?", DemoMode.STOPPED), CancelNavigation(context)]
    )

    goal = py_trees.composites.Sequence(name="GOAL?", memory=True)
    goal.add_children([ModeIs(context, "tryb GOAL?", DemoMode.GOAL), NavigateToGoal(context)])

    patrol = py_trees.composites.Sequence(name="PATROL fallback", memory=True)
    patrol.add_children([ModeIs(context, "tryb PATROL?", DemoMode.PATROL), PatrolCastle(context)])

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
            self.thread.join(timeout=2.0)
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
        helper_services.set_controller_frequency(demo_node)
        helper_services.publish_initial_pose(demo_node)

    navigator = BasicNavigator()
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = "map"
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.08
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)
    navigator.waitUntilNav2Active()
    navigator.clearAllCostmaps()

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
                stop_requested = True
                break

            time.sleep(args.period)
    finally:
        context.navigator.cancelTask()
        context.publish_stop(repeats=10)
        time.sleep(0.5)
        monitor_running = False
        monitor_thread.join(timeout=2.0)
        monitor_executor.remove_node(monitor)
        monitor_executor.shutdown()
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
    if fallback_seen and moving and commanded:
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
