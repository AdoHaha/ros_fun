"""Live action regressions; run as the desktop user in a sourced ROS container.

    ROS_DOMAIN_ID=96 python3 -m unittest test_action_workshop -v

The tests use a real, owned turtlesim. Do not run them alongside the exercise-7
notebook verifier in the same domain.
"""
import math
import time
import unittest

from action_workshop import close_action_lab, wait_action_future

try:
    from action_msgs.msg import GoalStatus
    from rclpy.action import ActionClient, ActionServer, GoalResponse
    from turtlesim.action import RotateAbsolute
    from turtlesim.msg import Pose
    from turtlesim.srv import TeleportAbsolute
    from intro_ros import IntroLab
except ModuleNotFoundError:
    IntroLab = None


@unittest.skipIf(IntroLab is None, 'requires workshop ROS container')
class LiveActionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sim_lab = IntroLab('action_test_simulator_observer')
        cls.addClassCleanup(cls.sim_lab.close)
        cls.sim_lab.start_turtlesim()
        if cls.sim_lab.process is None:
            raise RuntimeError('Refusing to test on an unowned turtlesim; select an unused ROS domain')

    def setUp(self):
        self.lab = IntroLab('action_test_client')
        self.client = ActionClient(self.lab.node, RotateAbsolute, '/turtle1/rotate_absolute')
        self.resources = {'lab': self.lab, 'action_client': self.client, 'proby_akcji': []}
        self.addCleanup(close_action_lab, self.resources)
        self.assertTrue(self.client.wait_for_server(timeout_sec=5))
        self.pose = None
        self.lab.node.create_subscription(Pose, '/turtle1/pose', self.receive_pose, 10)
        self.lab.call(TeleportAbsolute, '/turtle1/teleport_absolute',
                      TeleportAbsolute.Request(x=5.5, y=5.5, theta=0.0))
        self.lab.wait_for(lambda: self.pose is not None and abs(self.pose.theta) < 0.05)

    def receive_pose(self, message):
        self.pose = message

    def send(self, client, theta, feedback_callback=None):
        attempt = {'result_future': None}
        attempt['send_future'] = client.send_goal_async(
            RotateAbsolute.Goal(theta=float(theta)), feedback_callback=feedback_callback)
        self.resources['proby_akcji'].append(attempt)
        return attempt

    def accept(self, attempt):
        handle = wait_action_future(self.lab, attempt['send_future'])
        self.assertTrue(handle.accepted)
        attempt['result_future'] = handle.get_result_async()
        return handle

    def test_accepted_goal_feedback_and_real_final_heading(self):
        feedback = []
        attempt = self.send(self.client, math.pi / 2,
                            lambda msg: feedback.append(msg.feedback.remaining))
        self.accept(attempt)
        result = wait_action_future(self.lab, attempt['result_future'], timeout=8)
        self.lab.spin_for(0.1)
        self.assertEqual(result.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertGreater(len(feedback), 0)
        self.assertAlmostEqual(self.pose.theta, math.pi / 2, delta=0.05)
        # Jazzy turtlesim defines delta as start_theta - final_theta, i.e.
        # the angle back toward the starting heading, not final - start.
        self.assertAlmostEqual(result.result.delta, -math.pi / 2, delta=0.05)

    def test_cancellation_after_motion_requires_terminal_acknowledgement(self):
        feedback = []
        attempt = self.send(self.client, 3.0,
                            lambda msg: feedback.append(msg.feedback.remaining))
        handle = self.accept(attempt)
        self.lab.wait_for(lambda: feedback and self.pose.theta > 0.15, timeout=3)
        response = wait_action_future(self.lab, handle.cancel_goal_async())
        self.assertEqual(response.return_code, 0)
        self.assertEqual(len(response.goals_canceling), 1)
        self.assertEqual(bytes(response.goals_canceling[0].goal_id.uuid), bytes(handle.goal_id.uuid))
        result = wait_action_future(self.lab, attempt['result_future'])
        self.assertEqual(result.status, GoalStatus.STATUS_CANCELED)
        self.lab.spin_for(0.2)
        stopped = self.pose.theta
        self.lab.spin_for(0.2)
        self.assertAlmostEqual(self.pose.theta, stopped, delta=0.03)
        self.assertLess(stopped, 2.5)
        self.assertAlmostEqual(self.pose.angular_velocity, 0, delta=0.01)

    def test_cleanup_resolves_pending_acceptance_then_cancels_shared_server_goal(self):
        attempt = self.send(self.client, 3.0)
        self.assertFalse(attempt['send_future'].done())

        def record_result(future):
            handle = future.result()
            if handle.accepted:
                attempt['result_future'] = handle.get_result_async()
        attempt['send_future'].add_done_callback(record_result)
        close_action_lab(self.resources)
        self.assertFalse(self.lab.context.ok())
        self.assertTrue(attempt['send_future'].result().accepted)
        self.assertTrue(attempt['result_future'].done())
        self.assertEqual(attempt['result_future'].result().status, GoalStatus.STATUS_CANCELED)
        # The simulator belongs to a different lab: cleanup must leave it alive.
        self.assertIsNone(self.sim_lab.process.poll())
        close_action_lab(self.resources)  # Idempotent cleanup.

    def test_rejected_goal_is_distinct_from_execution_failure(self):
        server = ActionServer(self.lab.node, RotateAbsolute, '/action_test/rejected',
                              execute_callback=lambda handle: RotateAbsolute.Result(),
                              goal_callback=lambda goal: GoalResponse.REJECT)
        self.addCleanup(server.destroy)
        rejected_client = ActionClient(self.lab.node, RotateAbsolute, '/action_test/rejected')
        self.resources['nav_action_client'] = rejected_client
        self.assertTrue(rejected_client.wait_for_server(timeout_sec=3))
        attempt = self.send(rejected_client, 1.0)
        handle = wait_action_future(self.lab, attempt['send_future'])
        self.assertFalse(handle.accepted)
        self.assertIsNone(attempt['result_future'])

    def test_missing_server_and_pending_reply_have_bounded_waits(self):
        missing = ActionClient(self.lab.node, RotateAbsolute, '/action_test/not_running')
        self.resources['nav_action_client'] = missing
        started = time.monotonic()
        self.assertFalse(missing.wait_for_server(timeout_sec=0.2))
        attempt = self.send(missing, 1.0)
        with self.assertRaises(TimeoutError):
            wait_action_future(self.lab, attempt['send_future'], timeout=0.2)
        self.assertLess(time.monotonic() - started, 1.2)
        with self.assertWarnsRegex(RuntimeWarning, 'Timeout nie cofa celu'):
            close_action_lab(self.resources, timeout=0.2)
        self.assertFalse(self.lab.context.ok())


if __name__ == '__main__':
    unittest.main()
