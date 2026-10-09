"""Check instructor task answers and the actual notebook delivery callback.

These checks need no ROS installation. Function definitions are extracted from
the lesson sources, so the tests exercise the code students actually complete.
"""
import ast
import json
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

try:
    from intro_ros import IntroLab
    from turtlesim.srv import TeleportAbsolute
except ModuleNotFoundError:
    IntroLab = None


EXERCISES = Path(__file__).resolve().parent
ANSWERS = json.loads((EXERCISES / '.solutions' / 'intro_answers.json').read_text())


def load_definitions(source, namespace, constants=()):
    """Execute only definitions and named constants, never notebook setup."""
    tree = ast.parse(source)
    definitions = [statement for statement in tree.body if (
        isinstance(statement, (ast.FunctionDef, ast.AsyncFunctionDef)) or
        isinstance(statement, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id in constants
            for target in statement.targets))]
    exec(compile(ast.Module(body=definitions, type_ignores=[]), '<lesson>', 'exec'), namespace)
    return namespace


def lesson_function(name, namespace=None, constants=()):
    return load_definitions(ANSWERS[name], {'math': math, **(namespace or {})}, constants)


def notebook_function(filename, function_name, namespace):
    notebook = json.loads((EXERCISES / filename).read_text())
    sources = [''.join(cell['source']) for cell in notebook['cells']
               if cell['cell_type'] == 'code']
    for source in sources:
        if any(isinstance(statement, ast.FunctionDef) and statement.name == function_name
               for statement in ast.parse(source).body):
            return load_definitions(source, namespace)[function_name]
    raise AssertionError(f'Missing lesson function: {function_name}')


class InstructorTaskTests(unittest.TestCase):
    def test_numbered_messages_use_the_supplied_number(self):
        function = lesson_function('numbered_messages')['meldunek']
        for number in (1, 2, 3, 12, 100):
            with self.subTest(number=number):
                self.assertEqual(function(number), f'Paczka {number}')

    def test_distance_handles_axes_diagonals_and_different_targets(self):
        function = lesson_function('distance')['odleglosc']
        for x, y, target, expected in (
            (7, 7, (7, 7), 0), (8, 7, (7, 7), 1),
            (7, 6, (7, 7), 1), (10, 11, (7, 7), 5),
            (-3, -4, (0, 0), 5), (3, 3, (3, 7), 4),
            (7.4, 7.4, (7, 7), math.sqrt(0.32)),
        ):
            with self.subTest(position=(x, y), target=target):
                self.assertAlmostEqual(function(x, y, target), expected)

    def test_delivery_requires_a_position(self):
        function = lesson_function('delivery', constants=('CEL',))['ocen_dostawe']
        success, message = function(None)
        self.assertFalse(success)
        self.assertTrue(message)

    def test_delivery_uses_a_circle_in_both_coordinates(self):
        function = lesson_function('delivery', constants=('CEL',))['ocen_dostawe']
        for x, y, expected in (
            (5, 5, False), (7, 5, True), (7.3, 5, True),
            (7, 5.3, True), (7, 5.6, False),
            (7.4, 5.4, False), (6.6, 4.6, False),
        ):
            with self.subTest(position=(x, y)):
                success, message = function(SimpleNamespace(x=x, y=y))
                self.assertEqual(success, expected)
                self.assertTrue(message)

    def test_delivery_includes_radius_boundary_and_excludes_just_outside(self):
        function = lesson_function('delivery', constants=('CEL',))['ocen_dostawe']
        for x, y, expected in (
            (7.5, 5, True), (6.5, 5, True), (7, 5.5, True), (7, 4.5, True),
            (7.5001, 5, False), (7, 5.5001, False),
        ):
            with self.subTest(position=(x, y)):
                self.assertEqual(function(SimpleNamespace(x=x, y=y))[0], expected)


class DeliveryCallbackTests(unittest.TestCase):
    def setUp(self):
        self.namespace = lesson_function('delivery', constants=('CEL',))
        self.namespace.update(
            time=SimpleNamespace(monotonic=lambda: 100.0),
            dostawa_gotowa=True,
            stan={'pose': SimpleNamespace(x=7, y=5), 'time': 100.0},
            bilet={'wydany': False, 'numer': 0},
        )
        self.callback = notebook_function('5. ROS Service.ipynb', 'odbior_paczki', self.namespace)

    def call(self):
        response = SimpleNamespace(success=False, message='')
        self.assertIs(self.callback(SimpleNamespace(), response), response)
        return response

    def assert_no_receipt(self):
        response = self.call()
        self.assertFalse(response.success)
        self.assertTrue(response.message)
        self.assertEqual(self.namespace['bilet'], {'wydany': False, 'numer': 0})

    def test_unfinished_student_task_cannot_accept_delivery(self):
        self.namespace['dostawa_gotowa'] = False
        self.assert_no_receipt()

    def test_unknown_position_cannot_accept_delivery(self):
        self.namespace['stan'].update(pose=None, time=None)
        self.assert_no_receipt()

    def test_stale_position_cannot_accept_delivery(self):
        self.namespace['stan']['time'] = 99.49
        self.assert_no_receipt()

    def test_position_at_freshness_limit_can_accept_delivery(self):
        self.namespace['stan']['time'] = 99.5
        self.assertTrue(self.call().success)

    def test_current_but_distant_position_cannot_accept_delivery(self):
        self.namespace['stan']['pose'] = SimpleNamespace(x=7.4, y=5.4)
        self.assert_no_receipt()

    def test_one_delivery_issues_exactly_one_receipt(self):
        first = self.call()
        self.assertTrue(first.success)
        self.assertIn('#1', first.message)
        for _ in range(3):
            self.assertFalse(self.call().success)
            self.assertEqual(self.namespace['bilet'], {'wydany': True, 'numer': 1})

    def test_rejected_delivery_can_succeed_after_actual_state_changes(self):
        self.namespace['stan']['pose'] = SimpleNamespace(x=5, y=5)
        self.assert_no_receipt()
        self.namespace['stan']['pose'] = SimpleNamespace(x=7, y=5)
        self.assertTrue(self.call().success)
        self.assertEqual(self.namespace['bilet']['numer'], 1)


@unittest.skipIf(IntroLab is None, 'requires workshop ROS container and isolated ROS domain')
class SimulatorReuseTests(unittest.TestCase):
    def test_fresh_context_reuses_simulator_and_preserves_its_owner(self):
        owner = IntroLab('solvability_simulator_owner')
        self.addCleanup(owner.close)
        owner.start_turtlesim()
        self.assertIsNotNone(owner.process, 'run this test in an unused ROS domain')
        borrower = IntroLab('solvability_simulator_borrower')
        self.addCleanup(borrower.close)
        borrower.start_turtlesim()
        self.assertIsNone(borrower.process, 'a second simulator was started during discovery')
        borrower.close()
        self.assertIsNone(owner.process.poll(), 'borrower cleanup stopped the original simulator')
        result = owner.call(
            TeleportAbsolute, '/turtle1/teleport_absolute',
            TeleportAbsolute.Request(x=5.0, y=5.0, theta=0.0))
        self.assertIsInstance(result, TeleportAbsolute.Response)


if __name__ == '__main__':
    unittest.main()
