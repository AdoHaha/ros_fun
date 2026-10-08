"""Sensor geometry and live ROS lifecycle regressions; run in the sourced container."""
import math
import time
import unittest

try:
    from sensor_msgs.msg import LaserScan
    from std_srvs.srv import Trigger
    from geometry_msgs.msg import Twist, TwistStamped
    from intro_ros import IntroLab, front_distance
except ModuleNotFoundError:
    IntroLab = None


@unittest.skipIf(IntroLab is None, 'requires workshop ROS container')
class ScanGeometryTests(unittest.TestCase):
    def scan(self, ranges, start=-math.pi / 2, increment=math.pi / 2):
        return LaserScan(ranges=ranges, angle_min=start, angle_increment=increment,
                         range_min=0.1, range_max=4.0)

    def test_forward_is_not_first_sample(self):
        self.assertAlmostEqual(front_distance(self.scan([0.2, 1.5, 0.3])), 1.5)

    def test_nearest_in_actual_forward_cone(self):
        scan = self.scan([2.0, 0.6, 1.1, 0.4], start=-0.2, increment=0.2)
        self.assertAlmostEqual(front_distance(scan), 0.6)

    def test_unknown_and_invalid_are_not_clear(self):
        for value in [float('inf'), float('nan'), 0.0, 0.01, 5.0]:
            self.assertIsNone(front_distance(self.scan([value], start=0.0)))
        self.assertIsNone(front_distance(self.scan([])))

    def test_wraparound_and_negative_increment(self):
        scan = self.scan([1.3, 0.2], start=2 * math.pi, increment=-math.pi / 2)
        self.assertAlmostEqual(front_distance(scan), 1.3)


@unittest.skipIf(IntroLab is None, 'requires workshop ROS container')
class LiveLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.lab = IntroLab('intro_regression')
        self.addCleanup(self.lab.close)

    def test_timer_requires_spin_and_close_can_repeat(self):
        events = []
        self.lab.node.create_timer(0.05, lambda: events.append('tick'))
        time.sleep(0.1)
        self.assertEqual(events, [])
        self.lab.spin_for(0.15)
        self.assertGreater(len(events), 0)
        self.lab.close()
        self.lab.close()
        self.assertFalse(self.lab.context.ok())

    def test_real_request_response_in_one_executor(self):
        def reply(request, response):
            response.success = True
            response.message = 'parcel received'
            return response
        self.lab.node.create_service(Trigger, '/intro_test/receipt', reply)
        result = self.lab.call(Trigger, '/intro_test/receipt', Trigger.Request(), timeout=3)
        self.assertTrue(result.success)
        self.assertEqual(result.message, 'parcel received')

    def test_missing_service_has_deadline_and_no_leaked_client(self):
        before = len(list(self.lab.node.clients))
        started = time.monotonic()
        with self.assertRaises(TimeoutError):
            self.lab.call(Trigger, '/intro_test/missing', Trigger.Request(), timeout=0.2)
        self.assertLess(time.monotonic() - started, 1.0)
        self.assertEqual(len(list(self.lab.node.clients)), before)

    def test_visible_but_unspun_server_reply_has_deadline(self):
        other = IntroLab('unspun_intro_server')
        self.addCleanup(other.close)
        server = other.node.create_service(Trigger, '/intro_test/unspun', lambda req, res: res)
        discovery = self.lab.node.create_client(Trigger, '/intro_test/unspun')
        self.assertTrue(discovery.wait_for_service(timeout_sec=3))
        self.lab.node.destroy_client(discovery)
        with self.assertRaises(TimeoutError):
            self.lab.call(Trigger, '/intro_test/unspun', Trigger.Request(), timeout=0.3)
        self.assertEqual(len(list(self.lab.node.clients)), 0)
        other.node.destroy_service(server)

    def test_drive_stops_after_callback_failure(self):
        velocities = []
        self.lab.node.create_subscription(Twist, '/intro_test/velocity',
                                          lambda msg: velocities.append(msg.linear.x), 10)
        publisher = self.lab.node.create_publisher(Twist, '/intro_test/velocity', 10)
        self.lab.wait_for_subscriber(publisher)
        def fail():
            raise RuntimeError('injected callback failure')
        timer = self.lab.node.create_timer(0.15, fail)
        try:
            with self.assertRaisesRegex(RuntimeError, 'injected callback failure'):
                self.lab.drive(publisher, linear=1.0, seconds=0.5)
        finally:
            self.lab.node.destroy_timer(timer)
        self.lab.spin_for(0.1)
        self.assertIn(1.0, velocities)
        self.assertEqual(velocities[-1], 0.0)

    def test_stamped_velocity_uses_clock_and_nested_fields(self):
        commands = []
        self.lab.node.create_subscription(TwistStamped, '/intro_test/stamped', commands.append, 10)
        publisher = self.lab.node.create_publisher(TwistStamped, '/intro_test/stamped', 10)
        self.lab.wait_for_subscriber(publisher)
        self.lab.drive(publisher, linear=0.1, angular=0.2, seconds=0.2)
        self.lab.spin_for(0.1)
        self.assertTrue(any(msg.twist.linear.x == 0.1 for msg in commands))
        self.assertTrue(all(msg.header.stamp.sec > 0 for msg in commands))
        self.assertEqual(commands[-1].twist.linear.x, 0.0)
        self.assertEqual(commands[-1].twist.angular.z, 0.0)


if __name__ == '__main__':
    unittest.main()
