"""Validate the actual mission answer, atomic ROS updates and external CLI."""
import json
import math
from pathlib import Path
import unittest
from unittest.mock import patch

from intro_ros import IntroLab
from parameters_lab import run_parameter_cli
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.parameter import Parameter


HERE = Path(__file__).resolve().parent


def lesson_validator():
    source = json.loads((HERE / '.solutions/parameters_answers.json').read_text())['validator']
    namespace = dict(Parameter=Parameter, SetParametersResult=SetParametersResult, math=math)
    exec(source, namespace)
    assert namespace['walidacja_gotowa']
    return namespace['sprawdz_parametry']


class ParameterMissionTests(unittest.TestCase):
    def setUp(self):
        self.lab = IntroLab('parameter_mission_test')
        self.addCleanup(self.lab.close)
        self.node = self.lab.node
        self.node.declare_parameter('max_speed', 0.8)
        self.node.declare_parameter('arrival_radius', 0.5)
        self.node.add_on_set_parameters_callback(lesson_validator())

    def values(self):
        return [self.node.get_parameter(name).value for name in ('max_speed', 'arrival_radius')]

    def test_invalid_cli_deadline_never_starts_a_process(self):
        with patch('parameters_lab.subprocess.Popen') as launch:
            for value in (math.nan, math.inf, -math.inf, -1, 0):
                with self.assertRaises(ValueError):
                    run_parameter_cli(self.lab, ['get', '/missing', 'value'], timeout=value)
            launch.assert_not_called()

    def test_invalid_batch_preserves_both_values(self):
        result = self.node.set_parameters_atomically([
            Parameter('max_speed', value=0.4), Parameter('arrival_radius', value=-1.0)])
        self.assertFalse(result.successful)
        self.assertTrue(result.reason)
        self.assertEqual(self.values(), [0.8, 0.5])

    def test_limits_nonfinite_and_wrong_type_are_rejected(self):
        for name, value in [('max_speed', 0.0), ('max_speed', -1.0), ('max_speed', 2.0001),
                            ('max_speed', float('nan')), ('max_speed', float('inf')),
                            ('max_speed', 1), ('max_speed', 'fast'),
                            ('arrival_radius', 0.0999), ('arrival_radius', 1.0001)]:
            with self.subTest(name=name, value=value):
                result = self.node.set_parameters_atomically([Parameter(name, value=value)])
                self.assertFalse(result.successful)
                self.assertEqual(self.values(), [0.8, 0.5])

    def test_valid_boundaries_are_accepted(self):
        for speed, radius in [(2.0, 0.1), (0.0001, 1.0), (0.6, 0.4)]:
            result = self.node.set_parameters_atomically([
                Parameter('max_speed', value=speed), Parameter('arrival_radius', value=radius)])
            self.assertTrue(result.successful, result.reason)
            self.assertEqual(self.values(), [speed, radius])

    def test_single_parameter_batch_can_partially_succeed(self):
        results = self.node.set_parameters([
            Parameter('max_speed', value=0.4), Parameter('arrival_radius', value=-1.0)])
        self.assertEqual([result.successful for result in results], [True, False])
        self.assertEqual(self.values(), [0.4, 0.5])

    def test_remote_atomic_service_uses_same_validator(self):
        for radius, accepted in [(-1.0, False), (0.4, True)]:
            response = self.lab.call(SetParametersAtomically,
                '/parameter_mission_test/set_parameters_atomically',
                SetParametersAtomically.Request(parameters=[
                    Parameter('max_speed', value=0.6).to_parameter_msg(),
                    Parameter('arrival_radius', value=radius).to_parameter_msg()]))
            self.assertEqual(response.result.successful, accepted)
            self.assertEqual(self.values(), [0.6, 0.4] if accepted else [0.8, 0.5])

    def test_external_cli_accepts_good_and_rejects_bad_without_overwrite(self):
        result = run_parameter_cli(self.lab, ['set', '/parameter_mission_test', 'max_speed', '0.6'])
        self.assertIn('Set parameter successful', result.stdout)
        self.assertEqual(self.values(), [0.6, 0.5])
        result = run_parameter_cli(self.lab, ['set', '/parameter_mission_test', 'max_speed', '-1.0'])
        self.assertIn('Setting parameter failed', result.stdout + result.stderr)
        self.assertEqual(self.values(), [0.6, 0.5])

    def test_missing_server_times_out(self):
        with self.assertRaises(TimeoutError):
            run_parameter_cli(self.lab, ['get', '/missing_parameter_node', 'max_speed'], timeout=0.25)


if __name__ == '__main__':
    unittest.main()
