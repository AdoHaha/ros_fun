"""Deterministic checks for the student and instructor behaviour trees.

Run from the ROS workshop: python3 -m unittest discover -s exercises -p 'test_bt*.py' -v
"""

import ast
import json
from pathlib import Path
import unittest

import py_trees

import bt_workshop as lab

Status = py_trees.common.Status
EXERCISES = Path(__file__).resolve().parent


def notebook_builders(filename):
    """Load actual make_* definitions without executing demos or ROS startup."""
    namespace = {'py_trees': py_trees, 'lab': lab, 'Status': Status}
    notebook = json.loads((EXERCISES / filename).read_text())
    for cell in notebook['cells']:
        if cell['cell_type'] != 'code':
            continue
        module = ast.parse(''.join(cell['source']))
        module.body = [node for node in module.body
                       if isinstance(node, ast.FunctionDef) and node.name.startswith('make_')]
        if module.body:
            exec(compile(module, filename, 'exec'), namespace)
    return namespace


BUILDERS = notebook_builders('.solutions/10. Behavior Trees - ROS Applications - solutions.ipynb')
make_application = BUILDERS['make_application']


class WorkshopTests(unittest.TestCase):
    def mission(self, world, outcomes=None, retry_outcomes=None):
        root = make_application(
            world, lab.SimAction('Scan', world, outcomes=outcomes),
            lab.Led('Blue', world, 'blue'), lab.Record('Repair', world),
            lab.SimAction('Retry', world, outcomes=retry_outcomes),
        )
        self.addCleanup(root.stop, Status.INVALID)
        return root

    def test_instructor_checks(self):
        builders = notebook_builders('.solutions/9. Behavior Trees - solutions.ipynb')
        for check, name in [(lab.check_reactivity, 'make_reactive'),
                            (lab.check_parallel, 'make_parallel'),
                            (lab.check_event_memory, 'make_event')]:
            with self.subTest(name=name):
                check(builders[name])
        lab.check_mission(make_application)

    def test_student_starters_are_detected(self):
        builders = notebook_builders('9. Behavior Trees.ipynb')
        for check, name in [(lab.check_reactivity, 'make_reactive'),
                            (lab.check_parallel, 'make_parallel'),
                            (lab.check_event_memory, 'make_event')]:
            with self.subTest(name=name), self.assertRaises(AssertionError):
                check(builders[name])

    def test_each_reactive_memory_mutation_is_detected(self):
        for name in ['Snapshot then decisions', 'Priorities', 'Emergency']:
            def mutated(*args):
                root = make_application(*args)
                next(b for b in root.iterate() if b.name == name).memory = True
                return root
            with self.subTest(name=name), self.assertRaises(AssertionError):
                lab.check_mission(mutated)

    def test_previous_sensor_state_survives_every_exit(self):
        for exit_kind in ['success', 'failed', 'cancel', 'battery', 'stop']:
            with self.subTest(exit_kind=exit_kind):
                world = lab.World(scan_requested=True, sensors_enabled=True)
                failed = [Status.FAILURE] if exit_kind == 'failed' else None
                root = self.mission(world, failed, failed)
                lab.tick(root)
                if exit_kind == 'cancel':
                    world.cancel_requested = True
                    lab.tick(root)
                    self.assertEqual(world.result, 'cancelled')
                elif exit_kind == 'battery':
                    world.battery_low = True
                    lab.tick(root)
                    self.assertEqual(lab.count(world, 'Scan', 'cancel'), 1)
                elif exit_kind == 'stop':
                    root.stop(Status.INVALID)
                    self.assertEqual(lab.count(world, 'Scan', 'cancel'), 1)
                else:
                    lab.tick(root, 6)
                    self.assertEqual(world.result, exit_kind)
                self.assertTrue(world.sensors_enabled)
                self.assertNotEqual(world.led, 'blue')

    def test_priorities_interrupt_running_recovery(self):
        for reason in ['cancel', 'battery', 'both']:
            with self.subTest(reason=reason):
                world = lab.World(scan_requested=True)
                root = self.mission(world, [Status.FAILURE])
                lab.tick(root, 3)
                self.assertEqual(lab.count(world, 'Retry', 'start'), 1)
                world.cancel_requested = reason in ('cancel', 'both')
                world.battery_low = reason in ('battery', 'both')
                lab.tick(root)
                self.assertEqual(lab.count(world, 'Retry', 'cancel'), 1)
                self.assertFalse(world.sensors_enabled)
                self.assertNotEqual(world.led, 'blue')
                if reason == 'battery':
                    self.assertTrue(world.active)
                    self.assertIsNone(world.result)
                    world.battery_low = False
                    lab.tick(root, 3)
                    self.assertEqual(world.result, 'success')
                    self.assertEqual(lab.count(world, 'Scan', 'start'), 2)
                else:
                    self.assertFalse(world.active)
                    self.assertEqual(world.result, 'cancelled')
                self.assertEqual(lab.count(world, 'Repair', 'success'), 1)

    def test_cancel_during_alarm_discards_the_retained_request(self):
        world = lab.World(scan_requested=True)
        root = self.mission(world)
        lab.tick(root)
        world.battery_low = True
        lab.tick(root)
        world.cancel_requested = True
        lab.tick(root)
        self.assertEqual(world.result, 'cancelled')
        self.assertFalse(world.active)
        lab.tick(root)
        self.assertEqual(world.led, 'red')
        world.battery_low = False
        lab.tick(root, 6)
        self.assertEqual(lab.count(world, 'Scan', 'start'), 1)
        self.assertEqual(lab.count(world, 'result', 'cancelled'), 1)
        self.assertIsNone(world.led)

    def test_input_events_do_not_queue_or_leak_into_next_request(self):
        world = lab.World(scan_requested=True, cancel_requested=True)
        root = self.mission(world)
        lab.tick(root, 5)
        self.assertFalse(world.active)
        self.assertIsNone(world.result)
        self.assertEqual(lab.count(world, 'Scan', 'start'), 0)
        world.scan_requested = True
        lab.tick(root)
        world.scan_requested = True
        lab.tick(root, 8)
        self.assertEqual(lab.count(world, 'Scan', 'start'), 1)
        self.assertEqual(lab.count(world, 'result', 'success'), 1)
        world.scan_requested = True
        lab.tick(root, 8)
        self.assertEqual(lab.count(world, 'Scan', 'start'), 2)
        self.assertEqual(lab.count(world, 'result', 'success'), 2)


if __name__ == '__main__':
    unittest.main()
