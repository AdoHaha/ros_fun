"""Run from notebook root: python3 -m unittest discover -s robo_restaurant/tests -v."""
from dataclasses import replace
from pathlib import Path
import subprocess
import sys
import unittest
import xml.etree.ElementTree as ET
from robo_restaurant.planning import Action, State, execute, goal, solve, successors
from robo_restaurant.solutions.answers import audit_plan, energy_report, recover


class PlanningTests(unittest.TestCase):
    def test_known_optimal_costs(self):
        # >=6 travel edges (12), handling (10), >=one recharge (3): bound 25.
        for initial, expected in [(State(), 25), (State(battery=2), 28), (State(cooker_ok=False), 30)]:
            with self.subTest(initial=initial):
                plan, cost = solve(initial)
                final, actual = audit_plan(initial, plan)
                self.assertTrue(goal(final))
                self.assertEqual(cost, expected)
                self.assertEqual(actual, expected)

    def test_all_initial_energy_levels(self):
        for battery in range(9):
            for cooker_ok in (False, True):
                plan, _ = solve(State(battery=battery, cooker_ok=cooker_ok))
                self.assertIsNotNone(plan)
                final, _ = audit_plan(State(battery=battery, cooker_ok=cooker_ok), plan)
                self.assertTrue(goal(final))

    def test_every_reachable_transition_preserves_invariants(self):
        frontier, visited = [State()], set()
        while frontier:
            state = frontier.pop()
            if state in visited:
                continue
            visited.add(state)
            self.assertGreaterEqual(state.battery, 0)
            self.assertLessEqual(state.battery, 8)
            self.assertEqual(state.meals.count('carried'), int(state.carried >= 0))
            if state.carried >= 0:
                self.assertEqual(state.meals[state.carried], 'carried')
            for action, following, cost in successors(state):
                self.assertGreater(cost, 0)
                self.assertEqual(execute(state, action), following)
                for before, after in zip(state.meals, following.meals):
                    self.assertLessEqual(('ordered', 'ready', 'carried', 'served').index(before),
                                         ('ordered', 'ready', 'carried', 'served').index(after))
                frontier.append(following)
        self.assertGreater(len(visited), 100)

    def test_preconditions(self):
        cases = [(State(), Action('serve', '1')), (State(), Action('pick_up', '1')),
                 (State(location='kitchen', cooker_ok=False), Action('prepare', '1')),
                 (State(location='table_1', battery=0), Action('move', 'dock')),
                 (State(location='table_1'), Action('charge')),
                 (State(location='kitchen', carried=0, meals=('carried', 'ready')), Action('pick_up', '2')),
                 (State(location='table_2', carried=0, meals=('carried', 'ordered')), Action('serve', '1'))]
        for state, action in cases:
            with self.subTest(action=action), self.assertRaises(ValueError):
                execute(state, action)

    def test_failure_recovery_and_stale_rejection(self):
        plan, _ = solve(State())
        observed = State()
        while plan[0].name != 'prepare':
            observed = execute(observed, plan.pop(0))
        observed = replace(observed, cooker_ok=False)
        with self.assertRaises(ValueError):
            execute(observed, plan[0])
        result = recover(observed)
        self.assertEqual(result['plan'][0], Action('repair'))
        self.assertEqual(result['cost'], 28)
        final, cost = audit_plan(observed, result['plan'])
        self.assertTrue(goal(final))
        self.assertEqual(cost + 2, 30)

    def test_stranded_and_completed(self):
        stranded = State(location='table_1', battery=0)
        self.assertEqual(solve(stranded), (None, None))
        self.assertEqual(energy_report(stranded), {'feasible': False, 'cost': None, 'charges': None})
        self.assertEqual(recover(stranded), {'status': 'blocked', 'plan': None, 'cost': None})
        self.assertEqual(solve(State(meals=('served', 'served'))), ([], 0))

    def test_cli_scenarios(self):
        for scenario in ('normal', 'low_battery', 'cooker_failure'):
            result = subprocess.run([sys.executable, '-m', 'robo_restaurant.planning', '--scenario', scenario],
                                    capture_output=True, text=True, check=True, timeout=20)
            self.assertIn('SUCCESS', result.stdout)
            if scenario == 'cooker_failure':
                self.assertEqual(result.stdout.count('EVENT:'), 1)
                self.assertIn('repair', result.stdout)


class WorldTests(unittest.TestCase):
    def setUp(self):
        self.world = ET.parse(Path(__file__).resolve().parents[1] / 'worlds/restaurant.sdf').getroot().find('world')
        self.models = {m.attrib['name']: m for m in self.world.findall('model')}

    def test_scene_contract(self):
        self.assertEqual(self.world.attrib['name'], 'robo_restaurant')
        self.assertEqual(len(self.models), 21)
        for name in ('cook_robot', 'customer_robot_1', 'customer_robot_2', 'washing_station', 'cooker'):
            self.assertIn(name, self.models)
        self.assertTrue(all(m.findtext('static') == 'true' for m in self.models.values()))
        for name in ('charging_dock', 'kitchen_service_zone', 'table_1_service_zone', 'table_2_service_zone'):
            self.assertIsNone(self.models[name].find('.//collision'))
        self.assertIsNone(self.world.find('.//uri'))

    def test_service_pose_clearance(self):
        # Circle vs axis-aligned boxes: service clearance, not a navigation proof.
        for marker in ('charging_dock', 'kitchen_service_zone', 'table_1_service_zone', 'table_2_service_zone'):
            x, y, *_ = map(float, self.models[marker].findtext('pose').split())
            for name, model in self.models.items():
                if name == 'floor' or model.find('.//collision') is None:
                    continue
                ox, oy, *_ = map(float, model.findtext('pose').split())
                sx, sy, _ = map(float, model.findtext('.//collision/geometry/box/size').split())
                dx, dy = max(abs(x-ox)-sx/2, 0), max(abs(y-oy)-sy/2, 0)
                self.assertGreater(dx*dx + dy*dy, .35**2, (marker, name))


class GazeboAdapterTests(unittest.TestCase):
    def test_live_pose_parser_uses_actual_updates_and_zero_defaults(self):
        from robo_restaurant.gazebo import live_positions
        text = '''pose {
  name: "waiter_proxy"
  position {
    x: -2.4
    y: 2
    z: 0.35
  }
}
pose {
  name: "origin"
  position {
  }
}
'''
        self.assertEqual(live_positions(text), {'waiter_proxy': (-2.4, 2., .35), 'origin': (0., 0., 0.)})

    def test_expected_meal_and_fault_effects(self):
        from robo_restaurant.gazebo import expected_positions
        state = State(location='table_1', carried=0, meals=('carried', 'ready'), cooker_ok=False)
        poses = expected_positions(state)
        self.assertEqual(poses['meal_1'], (1.6, -2., .8))
        self.assertEqual(poses['meal_2'], (-3.5, 2., 1.1))
        self.assertEqual(poses['cooker_fault'][2], 1.3)
        poses = expected_positions(execute(state, Action('serve', '1')))
        self.assertEqual(poses['meal_1'], (3., -2., 1.2))


if __name__ == '__main__':
    unittest.main()
