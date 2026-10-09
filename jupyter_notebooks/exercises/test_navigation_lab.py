"""Navigation boundary regressions; live movement is checked by the notebook runner."""
import math
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from builtin_interfaces.msg import Time
from navigation_lab import NavigationLab, TaskResult, goal_pose, stop_owned_process


class NavigationBoundaryTests(unittest.TestCase):
    def make_lab(self):
        lab = NavigationLab.__new__(NavigationLab)
        lab.navigator = Mock()
        lab.tf_listener = Mock()
        lab.navigator.get_clock.return_value.now.return_value.to_msg.return_value = Time(sec=3)
        lab.processes = []
        lab.active = False
        lab.last_result = None
        lab.closed = False
        lab.owns_context = False
        return lab

    def test_pose_has_map_simulation_stamp_and_unit_orientation(self):
        lab = self.make_lab()
        for yaw in (0, math.pi/2, -math.pi/2, math.pi, 7):
            pose = goal_pose(lab.navigator, 0.5, -1.2, yaw)
            self.assertEqual(pose.header.frame_id, 'map')
            self.assertEqual(pose.header.stamp.sec, 3)
            q = pose.pose.orientation
            self.assertAlmostEqual(sum(value*value for value in (q.x, q.y, q.z, q.w)), 1)
            self.assertAlmostEqual(2*math.atan2(q.z, q.w) % (2*math.pi), yaw % (2*math.pi))

    def test_invalid_coordinates_never_make_navigation_goal(self):
        lab = self.make_lab()
        for values in ((math.nan, 0, 0), (0, math.inf, 0), (0, 0, -math.inf)):
            with self.assertRaises(ValueError):
                goal_pose(lab.navigator, *values)

    def test_accepted_goal_is_not_completed_and_is_sent_once(self):
        lab = self.make_lab()
        lab.navigator.goToPose.return_value = True
        self.assertTrue(lab.begin(goal_pose(lab.navigator, 0.5, -1, 0)))
        self.assertTrue(lab.active)
        self.assertIsNone(lab.last_result)
        lab.navigator.goToPose.assert_called_once()
        with self.assertRaises(RuntimeError):
            lab.begin(goal_pose(lab.navigator, 0, 0, 0))
        lab.navigator.goToPose.assert_called_once()

    def test_rejected_goal_cannot_count_as_delivery(self):
        lab = self.make_lab()
        lab.navigator.goToPose.return_value = False
        with self.assertRaises(RuntimeError):
            lab.begin(goal_pose(lab.navigator, 0, 0, 0))
        self.assertFalse(lab.active)
        self.assertIsNone(lab.last_result)

    def test_feedback_and_final_failure_remain_distinct(self):
        lab = self.make_lab()
        lab.active = True
        lab.navigator.isTaskComplete.side_effect = [False, False, True]
        lab.navigator.getFeedback.return_value = SimpleNamespace(distance_remaining=2)
        lab.navigator.getResult.return_value = TaskResult.FAILED
        feedback = Mock()
        self.assertEqual(lab.wait_result(feedback=feedback), TaskResult.FAILED)
        self.assertFalse(lab.active)
        feedback.assert_called_once()
        lab.navigator.goToPose.assert_not_called()

    def test_cancel_acknowledgement_does_not_replace_terminal_result(self):
        lab = self.make_lab()
        lab.active = True
        lab.navigator.isTaskComplete.side_effect = [False, False, True]
        lab.navigator.getResult.return_value = TaskResult.CANCELED
        self.assertEqual(lab.cancel(), TaskResult.CANCELED)
        self.assertEqual(lab.navigator.isTaskComplete.call_count, 3)
        lab.navigator.cancelTask.assert_called_once()
        self.assertFalse(lab.active)

    def test_timeout_requests_cancel(self):
        lab = self.make_lab()
        lab.active = True
        lab.navigator.isTaskComplete.return_value = False
        lab.cancel = Mock()
        with patch('navigation_lab.time.monotonic', side_effect=[0, 1]):
            with self.assertRaises(TimeoutError):
                lab.wait_result(timeout=0.1)
        lab.cancel.assert_called_once()

    def test_invalid_durations_cannot_disable_deadlines_or_send_cancel(self):
        lab = self.make_lab()
        lab.active = True
        for value in (math.nan, math.inf, -math.inf, -1):
            for operation in (lambda: lab.spin_for(value),
                              lambda: lab.wait_for(lambda: False, timeout=value),
                              lambda: lab.wait_result(timeout=value),
                              lambda: lab.cancel(timeout=value)):
                with self.assertRaises(ValueError):
                    operation()
        lab.navigator.cancelTask.assert_not_called()
        lab.navigator.isTaskComplete.assert_not_called()

    def test_close_stops_owned_processes_even_when_cancel_fails(self):
        lab = self.make_lab()
        process = Mock()
        lab.processes = [('gazebo', process, '/tmp/mock.log')]
        lab.cancel = Mock(side_effect=TimeoutError('cancel'))
        with patch('navigation_lab.stop_owned_process') as stop:
            with self.assertRaises(TimeoutError):
                lab.close()
            stop.assert_called_once_with(process)
        lab.navigator.destroy_node.assert_called_once()
        lab.tf_listener.unregister.assert_called_once()
        self.assertTrue(lab.closed)

    def test_launcher_exit_does_not_leave_owned_children(self):
        process = Mock(pid=12345)
        with patch('navigation_lab.os.killpg', side_effect=[None, None, ProcessLookupError]) as kill:
            stop_owned_process(process)
        self.assertEqual(kill.call_count, 3)

    def test_existing_world_is_preserved_without_motion_or_launch(self):
        nav = Mock()
        nav.get_node_names_and_namespaces.return_value = [('amcl', '/'), ('courier_navigator', '/')]
        nav.count_publishers.return_value = 1
        with patch('navigation_lab.rclpy.ok', return_value=True), \
             patch('navigation_lab.WorkshopNavigator', return_value=nav), \
             patch.object(NavigationLab, 'spin_for'), \
             patch.object(NavigationLab, '_launch') as launch:
            with self.assertRaisesRegex(RuntimeError, 'Gazebo/Nav2'):
                NavigationLab()
        launch.assert_not_called()
        nav.create_publisher.return_value.publish.assert_not_called()
        nav.destroy_node.assert_called_once()


if __name__ == '__main__':
    unittest.main()
