"""Fast Nav2 boundary tests: no simulator, ROS daemon, or action servers.

    python3 -m unittest -v test_nav_bt

The notebook imports make_fake_demo() to reproduce these traces interactively.
"""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from builtin_interfaces.msg import Time
import py_trees
from nav2_simple_commander.robot_navigator import TaskResult

import trees_nav

Status = py_trees.common.Status


class FakeNavigator:
    """Record the boundary API; outcomes are controlled by the learner/test."""

    def __init__(self):
        self.calls = []
        self.result = None
        self.accept_goals = True

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time))

    def goToPose(self, pose):
        self.calls.append(("send", pose.pose.position.x, pose.pose.position.y))
        self.result = None
        return self.accept_goals

    def cancelTask(self):
        self.calls.append(("cancel",))
        self.result = TaskResult.CANCELED

    def isTaskComplete(self):
        return self.result is not None

    def getFeedback(self):
        return None

    def getResult(self):
        return self.result

    def clearAllCostmaps(self):
        self.calls.append(("clear",))

    def complete(self, result=TaskResult.SUCCEEDED):
        self.result = result


class FakePublisher:
    def __init__(self, navigator):
        self.navigator = navigator

    def publish(self, message):
        self.navigator.calls.append(("zero_velocity",))


def make_fake_demo():
    navigator = FakeNavigator()
    context = trees_nav.DemoContext(
        node=navigator, navigator=navigator, state=trees_nav.DemoState(),
        waypoints=[(1.0, 0.0, 0.0), (2.0, 0.0, 0.0)], goal=(9.0, 0.0, 0.0),
        cmd_vel_publisher=FakePublisher(navigator),
    )
    return context, navigator, trees_nav.make_tree(context)


class StartupWaitTests(unittest.TestCase):
    """Exercise the real startup polling with controlled DDS replies and wall clock."""

    def fake_navigator(self, futures):
        nav = Mock()
        nav.create_client.return_value.wait_for_service.return_value = True
        nav.create_client.return_value.call_async.side_effect = futures
        return nav

    def test_missing_action_acknowledgement_is_bounded(self):
        future = Mock()
        future.done.return_value = False
        with patch.object(trees_nav.rclpy, "spin_until_future_complete") as spin:
            with self.assertRaisesRegex(TimeoutError, "goal acknowledgement"):
                trees_nav.WorkshopNavigator._await_action_reply(Mock(), future, "goal acknowledgement")
        self.assertEqual(spin.call_args.kwargs["timeout_sec"], 2.0)

    def test_action_server_discovery_has_deadline(self):
        nav = Mock()
        nav.nav_to_pose_client.wait_for_server.return_value = False
        with patch.object(trees_nav.time, "monotonic", side_effect=[0.0, 0.0, 0.0, 5.1]):
            with self.assertRaisesRegex(TimeoutError, "server unavailable"):
                trees_nav.WorkshopNavigator.goToPose(nav, trees_nav.PoseStamped())
        nav.nav_to_pose_client.send_goal_async.assert_not_called()

    def test_late_goal_acceptance_is_canceled_after_ack_timeout(self):
        nav = Mock()
        nav.nav_to_pose_client.wait_for_server.return_value = True
        nav._await_action_reply.side_effect = TimeoutError("late reply")
        future = nav.nav_to_pose_client.send_goal_async.return_value
        with patch.object(trees_nav.time, "monotonic", return_value=0.0):
            with self.assertRaises(TimeoutError):
                trees_nav.WorkshopNavigator.goToPose(nav, trees_nav.PoseStamped())
        future.add_done_callback.assert_called_once_with(nav._cancel_late_goal)
        # Exercise the late completion hook with an actual accepted fake handle.
        late = Mock()
        late.cancelled.return_value = False
        late.exception.return_value = None
        late.result.return_value.accepted = True
        trees_nav.WorkshopNavigator._cancel_late_goal(late)
        late.result.return_value.cancel_goal_async.assert_called_once()

    def test_successful_goal_sets_handle_and_result_future(self):
        nav = Mock()
        nav.nav_to_pose_client.wait_for_server.return_value = True
        handle = Mock()
        handle.accepted = True
        nav._await_action_reply.return_value = handle
        pose = trees_nav.PoseStamped()
        with patch.object(trees_nav.time, "monotonic", return_value=0.0):
            self.assertTrue(trees_nav.WorkshopNavigator.goToPose(nav, pose, "my_tree.xml"))
        self.assertIs(nav.goal_handle, handle)
        self.assertIs(nav.result_future, handle.get_result_async.return_value)
        goal = nav.nav_to_pose_client.send_goal_async.call_args.args[0]
        self.assertEqual(goal.behavior_tree, "my_tree.xml")
        self.assertEqual(goal.pose, pose)

    def test_cancel_acknowledgement_uses_bounded_wait(self):
        nav = Mock()
        trees_nav.WorkshopNavigator.cancelTask(nav)
        nav._await_action_reply.assert_called_once_with(
            nav.goal_handle.cancel_goal_async.return_value, "cancellation acknowledgement")

    def test_clear_all_costmaps_share_one_deadline(self):
        nav = Mock()
        with patch.object(trees_nav.time, "monotonic", side_effect=[0.0, 1.0, 6.0]), \
             patch.object(trees_nav, "bounded_service_call") as call:
            trees_nav.WorkshopNavigator.clearAllCostmaps(nav)
        self.assertEqual([c.kwargs["timeout_sec"] for c in call.call_args_list], [7.0, 2.0])

    def test_costmap_call_retries_lost_reply_with_bounded_spins(self):
        pending = Mock()
        pending.done.return_value = False
        completed = Mock()
        completed.done.return_value = True
        completed.exception.return_value = None
        completed.result.return_value = object()
        client = Mock()
        client.wait_for_service.return_value = True
        client.call_async.side_effect = [pending, completed]
        with patch.object(trees_nav.time, "monotonic", return_value=0.0), \
             patch.object(trees_nav.rclpy, "spin_until_future_complete") as spin:
            result = trees_nav.bounded_service_call(Mock(), client, object(), "costmap clearing")
        self.assertIs(result, completed.result.return_value)
        self.assertEqual(client.call_async.call_count, 2)
        self.assertEqual(spin.call_args.kwargs["timeout_sec"], 2.0)
        client.remove_pending_request.assert_called_once_with(pending)

    def test_service_deadline_catches_missing_controller_reply(self):
        pending = Mock()
        pending.done.return_value = False
        client = Mock()
        client.wait_for_service.return_value = True
        client.call_async.return_value = pending
        with patch.object(trees_nav.time, "monotonic", side_effect=[0.0, 0.0, 0.0, 0.0, 2.1]), \
             patch.object(trees_nav.rclpy, "spin_until_future_complete"):
            with self.assertRaisesRegex(TimeoutError, "controller frequency.*reply"):
                trees_nav.bounded_service_call(Mock(), client, object(), "controller frequency", timeout_sec=2.0)
        pending.cancel.assert_called_once()

    def test_costmap_public_methods_use_bounded_service_helper(self):
        nav = Mock()
        with patch.object(trees_nav, "bounded_service_call") as call:
            trees_nav.WorkshopNavigator.clearLocalCostmap(nav)
            trees_nav.WorkshopNavigator.clearGlobalCostmap(nav)
        self.assertEqual(call.call_args_list[0].args[1], nav.clear_costmap_local_srv)
        self.assertEqual(call.call_args_list[1].args[1], nav.clear_costmap_global_srv)
        self.assertEqual(call.call_count, 2)

    def test_controller_configuration_preserves_parameter_and_destroys_client(self):
        node = Mock()
        response = SimpleNamespace(results=[SimpleNamespace(successful=True, reason="")])
        with patch.object(trees_nav, "bounded_service_call", return_value=response) as call:
            trees_nav.configure_controller_frequency(node)
        parameter = call.call_args.args[2].parameters[0]
        self.assertEqual(parameter.name, "controller_frequency")
        self.assertEqual(parameter.value.double_value, 5.0)
        node.destroy_client.assert_called_once_with(node.create_client.return_value)

    def test_lost_lifecycle_reply_is_discarded_and_retried(self):
        pending = Mock()
        pending.done.return_value = False
        active = Mock()
        active.done.return_value = True
        active.exception.return_value = None
        active.result.return_value = SimpleNamespace(current_state=SimpleNamespace(label="active"))
        nav = self.fake_navigator([pending, active])
        with patch.object(trees_nav.time, "monotonic", return_value=0.0), \
             patch.object(trees_nav.rclpy, "spin_until_future_complete") as spin:
            trees_nav.WorkshopNavigator._wait_for_active_node(nav, "amcl", deadline=10.0)
        self.assertEqual(nav.create_client.return_value.call_async.call_count, 2)
        nav.create_client.return_value.remove_pending_request.assert_called_once_with(pending)
        pending.cancel.assert_called_once()
        self.assertEqual(spin.call_args.kwargs["timeout_sec"], 2.0)
        nav.destroy_client.assert_called_once_with(nav.create_client.return_value)

    def test_missing_lifecycle_reply_has_a_wall_clock_deadline(self):
        pending = Mock()
        pending.done.return_value = False
        nav = self.fake_navigator([pending])
        with patch.object(trees_nav.time, "monotonic", side_effect=[0.0, 0.0, 0.0, 2.1]), \
             patch.object(trees_nav.rclpy, "spin_until_future_complete"):
            with self.assertRaisesRegex(TimeoutError, "amcl/get_state.*no lifecycle response"):
                trees_nav.WorkshopNavigator._wait_for_active_node(nav, "amcl", deadline=2.0)
        pending.cancel.assert_called_once()
        nav.destroy_client.assert_called_once()

    def test_missing_amcl_pose_has_a_wall_clock_deadline(self):
        nav = Mock()
        nav.initial_pose_received = False
        with patch.object(trees_nav.time, "monotonic", side_effect=[0.0, 0.0, 2.1]), \
             patch.object(trees_nav.rclpy, "spin_once") as spin:
            with self.assertRaisesRegex(TimeoutError, "AMCL pose"):
                trees_nav.WorkshopNavigator._wait_for_initial_pose(nav, deadline=2.0)
        nav._setInitialPose.assert_called_once()
        self.assertEqual(spin.call_count, 2)
        self.assertEqual(spin.call_args.kwargs["timeout_sec"], 0.1)



class NavigationTreeTests(unittest.TestCase):
    def setUp(self):
        self.context, self.nav, self.tree = make_fake_demo()

    def delay_cancellation(self):
        def request_cancel():
            self.nav.calls.append(("cancel",))
            self.nav.result = None  # Acknowledged, but old action has not terminated.
        self.nav.cancelTask = request_cancel

    def test_handoff_waits_for_canceled_action_terminal_result(self):
        self.tree.tick()
        old_leaf = self.context.navigation_owner
        self.delay_cancellation()
        self.context.request_goal()
        self.tree.tick()
        pending_leaf = self.context.navigation_owner
        self.assertEqual(self.nav.calls, [("send", 1.0, 0.0), ("cancel",)])
        self.assertEqual(old_leaf.status, Status.INVALID)
        self.assertFalse(old_leaf.goal_sent)
        self.assertTrue(pending_leaf.goal_pending)
        for _ in range(3):
            self.tree.tick()
        self.assertEqual(len(self.nav.calls), 2)
        self.nav.complete(TaskResult.CANCELED)
        self.tree.tick()
        self.assertEqual(self.nav.calls[-1], ("send", 9.0, 0.0))
        self.assertFalse(self.context.cancellation_pending)
        self.assertTrue(pending_leaf.goal_sent)
        self.assertFalse(pending_leaf.goal_pending)

    def test_stop_during_handoff_never_sends_pending_goal(self):
        self.tree.tick()
        self.delay_cancellation()
        self.context.request_goal()
        self.tree.tick()
        pending_leaf = self.context.navigation_owner
        self.context.request_stop()
        self.tree.tick()
        self.assertEqual(self.nav.calls, [("send", 1.0, 0.0), ("cancel",), ("zero_velocity",)])
        self.assertIsNone(self.context.navigation_owner)
        self.assertFalse(pending_leaf.goal_pending)
        self.assertTrue(self.context.cancellation_pending)
        self.nav.complete(TaskResult.CANCELED)
        self.tree.tick()
        self.assertEqual(len(self.nav.calls), 3)
        # Resuming patrol drains the retained cancellation before sending a new goal.
        self.context.request_patrol()
        self.tree.tick()
        self.assertEqual(self.nav.calls[-1], ("send", 1.0, 0.0))
        self.assertFalse(self.context.cancellation_pending)

    def test_new_intent_during_handoff_retains_old_cancellation(self):
        self.tree.tick()
        self.delay_cancellation()
        self.context.request_goal()
        self.tree.tick()
        self.context.request_patrol()
        self.tree.tick()
        self.assertEqual(self.nav.calls, [("send", 1.0, 0.0), ("cancel",)])
        self.assertTrue(self.context.cancellation_pending)
        self.nav.complete(TaskResult.CANCELED)
        self.tree.tick()
        self.assertEqual(self.nav.calls[-1], ("send", 1.0, 0.0))
        self.assertFalse(self.context.cancellation_pending)

    def test_setup_configures_sim_clock_before_stamping_initial_pose(self):
        """System-clock stamps cannot be transformed against Gazebo's TF clock."""
        class SetupNavigator(FakeNavigator):
            def __init__(self):
                super().__init__()
                self.sim_clock_enabled = False
                self.lifecycle = []

            def set_parameters(self, parameters):
                self.sim_clock_enabled = any(
                    p.name == "use_sim_time" and p.value is True for p in parameters
                )
                self.lifecycle.append("enable_sim_clock")

            def get_clock(self):
                if not self.sim_clock_enabled:
                    raise AssertionError("Initial pose stamped before enabling simulation time")
                self.lifecycle.append("read_sim_clock")
                return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=17)))

            def setInitialPose(self, pose):
                self.lifecycle.append("initial_pose")
                self.initial_pose = pose

            def waitUntilNav2Active(self):
                self.lifecycle.append("wait_active")

        navigator = SetupNavigator()
        node = Mock()
        with patch.object(trees_nav.rclpy, "init"), \
             patch.object(trees_nav, "DemoNode", return_value=node), \
             patch.object(trees_nav, "WorkshopNavigator", return_value=navigator):
            _, context = trees_nav.setup_navigation()
        self.assertTrue(navigator.sim_clock_enabled)
        self.assertEqual(navigator.lifecycle[:3],
                         ["enable_sim_clock", "read_sim_clock", "initial_pose"])
        self.assertEqual(navigator.initial_pose.header.stamp.sec, 17)
        self.assertIs(context.navigator, navigator)

    def test_callbacks_only_change_intents(self):
        self.tree.tick()
        calls = list(self.nav.calls)
        self.context.request_goal()
        self.context.request_stop()
        self.context.request_patrol()
        self.assertEqual(self.nav.calls, calls)

    def test_running_goal_is_sent_once(self):
        for _ in range(5):
            self.tree.tick()
        self.assertEqual(self.nav.calls, [("send", 1.0, 0.0)])
        self.assertEqual(self.tree.root.status, Status.RUNNING)

    def test_patrol_goal_priority_transfer_and_stop(self):
        self.tree.tick()
        old = self.context.navigation_owner
        self.context.request_goal()
        self.tree.tick()
        self.assertEqual(self.nav.calls, [("send", 1.0, 0.0), ("cancel",), ("send", 9.0, 0.0)])
        self.assertEqual(old.status, Status.INVALID)
        self.assertFalse(old.goal_sent)
        new = self.context.navigation_owner
        self.context.request_stop()
        self.tree.tick()
        self.assertEqual(self.nav.calls[-2:], [("cancel",), ("zero_velocity",)])
        self.assertIsNone(self.context.navigation_owner)
        self.assertFalse(new.goal_sent)
        calls = list(self.nav.calls)
        self.tree.tick()
        self.assertEqual(self.nav.calls, calls)

    def test_clear_request_interrupts_goal_and_restarts_patrol(self):
        self.context.request_goal()
        self.tree.tick()
        old = self.context.navigation_owner
        self.context.request_patrol()
        self.tree.tick()
        self.assertEqual(self.nav.calls[-2:], [("cancel",), ("send", 1.0, 0.0)])
        self.assertFalse(old.goal_sent)
        self.assertEqual(old.status, Status.INVALID)

    def test_goal_success_returns_to_patrol_on_next_tick(self):
        self.context.request_goal()
        self.tree.tick()
        self.nav.complete()
        self.tree.tick()
        self.assertEqual(self.context.state.snapshot()[0], trees_nav.DemoMode.PATROL)
        self.assertIsNone(self.context.navigation_owner)
        self.tree.tick()
        self.assertEqual(self.nav.calls, [("send", 9.0, 0.0), ("send", 1.0, 0.0)])

    def test_failed_goal_recovers_once_then_succeeds(self):
        self.context.request_goal()
        self.tree.tick()
        self.nav.complete(TaskResult.FAILED)
        self.tree.tick()
        self.assertEqual(self.nav.calls, [("send", 9.0, 0.0), ("clear",), ("send", 9.0, 0.0)])
        self.tree.tick()
        self.tree.tick()
        self.assertEqual(len(self.nav.calls), 3, "memory keeps the active retry, not the first attempt")
        self.nav.complete()
        self.tree.tick()
        self.assertEqual(self.context.state.snapshot()[0], trees_nav.DemoMode.PATROL)

    def test_failure_after_retry_stops_without_infinite_resends(self):
        self.context.request_goal()
        self.tree.tick()
        for _ in range(2):
            self.nav.complete(TaskResult.FAILED)
            self.tree.tick()
        self.assertEqual(self.context.state.snapshot()[0], trees_nav.DemoMode.STOPPED)
        for _ in range(4):
            self.tree.tick()
        self.assertEqual(self.nav.calls, [("send", 9.0, 0.0), ("clear",), ("send", 9.0, 0.0), ("zero_velocity",)])

    def test_patrol_advances_only_on_success_and_counts_laps(self):
        self.tree.tick()
        self.nav.complete(TaskResult.FAILED)
        self.tree.tick()
        self.assertEqual(self.context.patrol_waypoint_index, 0)
        self.nav.complete()
        self.tree.tick()
        self.assertEqual(self.context.patrol_waypoint_index, 1)
        self.tree.tick()
        self.assertEqual(self.nav.calls[-1], ("send", 2.0, 0.0))
        self.nav.complete()
        self.tree.tick()
        self.assertEqual(self.context.state.snapshot()[2], 1)

    def test_patrol_failure_after_retry_stops(self):
        self.tree.tick()
        for _ in range(2):
            self.nav.complete(TaskResult.FAILED)
            self.tree.tick()
        self.tree.tick()
        self.assertEqual(self.context.state.snapshot()[0], trees_nav.DemoMode.STOPPED)
        self.assertIsNone(self.context.navigation_owner)
        self.assertEqual(self.nav.calls[-1], ("zero_velocity",))

    def test_rejected_goals_are_failures_not_stale_result_reads(self):
        self.context.request_goal()
        self.nav.accept_goals = False
        self.tree.tick()
        self.assertEqual(self.context.state.snapshot()[0], trees_nav.DemoMode.STOPPED)
        self.assertIsNone(self.context.navigation_owner)
        self.assertEqual(self.nav.calls, [("send", 9.0, 0.0), ("clear",), ("send", 9.0, 0.0)])

    def test_root_memory_mutation_hides_stop(self):
        self.tree.root.memory = True  # Deliberately incorrect variant for the exercise.
        self.tree.tick()
        self.context.request_stop()
        self.tree.tick()
        self.assertEqual(self.nav.calls, [("send", 1.0, 0.0), ("cancel",)])
        self.assertEqual(self.tree.root.status, Status.FAILURE)
        self.assertIsNone(self.context.navigation_owner)
        # The guard still invalidates patrol, so it cancels, but STOP was skipped.

    def test_goal_guard_memory_mutation_ignores_clear_request(self):
        self.tree.root.children[1].memory = True  # Deliberately incorrect guard sequence.
        self.context.request_goal()
        self.tree.tick()
        self.context.request_patrol()
        self.tree.tick()
        self.assertEqual(self.nav.calls, [("send", 9.0, 0.0)])
        self.assertEqual(self.tree.root.status, Status.RUNNING)
        self.assertIsNotNone(self.context.navigation_owner)

    def test_shutdown_invalidates_and_cancels_once(self):
        self.tree.tick()
        owner = self.context.navigation_owner
        self.tree.root.stop(Status.INVALID)
        self.assertEqual(self.nav.calls[-1], ("cancel",))
        self.assertFalse(owner.goal_sent)
        self.assertIsNone(self.context.navigation_owner)

    def test_completed_request_does_not_overwrite_new_stop_intent(self):
        self.context.request_goal()
        self.assertFalse(self.context.state.transition(
            trees_nav.DemoMode.PATROL, trees_nav.DemoMode.STOPPED, "old result", "old_result"))
        self.assertEqual(self.context.state.snapshot()[0], trees_nav.DemoMode.GOAL)


if __name__ == "__main__":
    unittest.main()
