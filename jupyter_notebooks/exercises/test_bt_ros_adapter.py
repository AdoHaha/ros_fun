"""ROS adapter regressions with controllable futures; no ROS graph is started."""

from concurrent.futures import Future
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from action_msgs.msg import GoalStatus
from py_trees_ros_interfaces.action import Rotate

import bt_workshop as lab
import bt_workshop_ros as adapter
from test_bt_workshop import make_application, Status


class GoalHandle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.result_future = Future()
        self.cancellations = []

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        future = Future()
        self.cancellations.append(future)
        return future


class ActionClientTests(unittest.TestCase):
    def setUp(self):
        self.behaviour = adapter.CancellableActionClient(
            name='Scan', action_type=Rotate, action_name='/rotate', action_goal=Rotate.Goal(),
        )
        self.behaviour.node = Mock()
        self.requests = []
        def send(*args, **kwargs):
            future = Future()
            self.requests.append(future)
            return future
        self.behaviour.action_client = Mock(send_goal_async=send)

    def tick(self):
        self.behaviour.tick_once()
        return self.behaviour.status

    def test_pending_acceptance_sends_once_and_cancels_after_interruption(self):
        for _ in range(5):
            self.assertEqual(self.tick(), Status.RUNNING)
        self.assertEqual(len(self.requests), 1)
        self.behaviour.stop(Status.INVALID)
        handle = GoalHandle()
        self.requests[0].set_result(handle)
        self.assertEqual(len(handle.cancellations), 1)
        self.assertIsNone(self.behaviour.goal_handle)
        self.assertIsNone(self.behaviour.get_result_future)
        self.assertEqual(self.behaviour.status, Status.INVALID)

    def test_accepted_goal_is_cancelled_once(self):
        self.tick()
        handle = GoalHandle()
        self.requests[0].set_result(handle)
        self.behaviour.stop(Status.INVALID)
        self.behaviour.stop(Status.INVALID)
        self.assertEqual(len(handle.cancellations), 1)

    def test_stale_acceptance_cancels_old_goal_without_replacing_current_goal(self):
        self.tick()
        old_request = self.requests[-1]
        self.behaviour.stop(Status.INVALID)
        self.tick()
        current_handle = GoalHandle()
        self.requests[-1].set_result(current_handle)
        stale_handle = GoalHandle()
        old_request.set_result(stale_handle)
        self.assertEqual(len(stale_handle.cancellations), 1)
        self.assertEqual(current_handle.cancellations, [])
        self.assertIs(self.behaviour.goal_handle, current_handle)
        self.assertIs(self.behaviour.get_result_future, current_handle.result_future)
        self.assertEqual(self.tick(), Status.RUNNING)

    def test_stale_result_cannot_complete_or_fail_the_new_goal(self):
        self.tick()
        old_handle = GoalHandle()
        self.requests[-1].set_result(old_handle)
        self.behaviour.stop(Status.INVALID)
        self.tick()
        current_handle = GoalHandle()
        self.requests[-1].set_result(current_handle)
        old_handle.result_future.set_result(SimpleNamespace(status=GoalStatus.STATUS_ABORTED))
        self.assertIsNone(self.behaviour.result_status)
        self.assertEqual(self.tick(), Status.RUNNING)
        current_handle.result_future.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
        self.assertEqual(self.tick(), Status.SUCCESS)
        self.assertEqual(current_handle.cancellations, [])

    def test_stale_rejection_does_not_reject_the_new_goal(self):
        self.tick()
        old_request = self.requests[-1]
        self.behaviour.stop(Status.INVALID)
        self.tick()
        old_request.set_result(GoalHandle(accepted=False))
        self.assertIsNone(self.behaviour.goal_handle)
        self.assertEqual(self.tick(), Status.RUNNING)
        self.requests[-1].set_result(GoalHandle(accepted=False))
        self.assertEqual(self.tick(), Status.FAILURE)


class RosOutputTests(unittest.TestCase):
    def setUp(self):
        self.node = Mock()
        self.world = None
        self.outputs = []
        def publish(message):
            self.outputs.append(message.data)
            self.world.events.append(('publish', message.data))
        self.node.create_publisher.return_value.publish.side_effect = publish
        def builder(world, scan, led, repair, retry):
            self.world = world
            scan.world = retry.world = world
            return make_application(world, scan, led, repair, retry)
        def action(**kwargs):
            return lab.SimAction(kwargs['name'], None, ticks=3)
        self.ok = self.enterContext(patch.object(adapter.rclpy, 'ok', return_value=True))
        self.enterContext(patch.object(adapter.rclpy, 'create_node', return_value=self.node))
        self.enterContext(patch.object(adapter.rclpy, 'spin_once'))
        self.enterContext(patch.object(adapter, 'CancellableActionClient', side_effect=action))
        self.ros_lab = adapter.RosLab(builder)
        self.addCleanup(self.close_lab)

    def close_lab(self):
        self.ok.return_value = False  # No ROS callbacks or real waiting in these tests.
        self.ros_lab.close()

    def test_alarm_publishes_after_blue_cleanup_and_resume_selects_blue(self):
        self.world.battery_low = False
        self.world.scan_requested = True
        self.ros_lab.step()
        self.assertEqual(self.outputs, ['blue'])
        self.world.battery_low = True
        self.ros_lab.step()
        self.assertEqual(self.outputs, ['blue', 'red'])
        self.assertLess(self.world.events.index(('Blue', 'clear')),
                        self.world.events.index(('publish', 'red')))
        self.assertEqual(lab.count(self.world, 'Scan', 'cancel'), 1)
        self.world.battery_low = False
        self.ros_lab.step()
        self.assertEqual(self.outputs, ['blue', 'red', 'blue'])
        self.assertEqual(lab.count(self.world, 'Scan', 'start'), 2)
        self.node.create_publisher.assert_called_once()

    def test_completion_cancel_and_close_publish_clear(self):
        self.world.battery_low = False
        self.world.scan_requested = True
        for _ in range(3):
            self.ros_lab.step()
        self.assertEqual(self.world.result, 'success')
        self.assertEqual(self.outputs, ['blue', 'blue', ''])
        self.world.scan_requested = True
        self.ros_lab.step()
        self.world.cancel_requested = True
        self.ros_lab.step()
        self.assertEqual(self.world.result, 'cancelled')
        self.assertEqual(self.outputs[-2:], ['blue', ''])
        self.close_lab()
        outputs_after_close = list(self.outputs)
        self.close_lab()
        self.assertEqual(self.outputs, outputs_after_close)
        self.assertEqual(self.outputs[-1], '')
        self.node.destroy_node.assert_called_once()


if __name__ == '__main__':
    unittest.main()
