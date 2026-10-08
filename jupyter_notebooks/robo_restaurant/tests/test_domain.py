"""Public domain smoke tests; run from jupyter_notebooks with unittest discover."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
from robo_restaurant.planning import Action, State, execute, solve, successors


class DomainTests(unittest.TestCase):
    def test_demo_costs(self):
        for state, expected in [(State(), 25), (State(battery=2), 28), (State(cooker_ok=False), 30)]:
            self.assertEqual(solve(state)[1], expected)

    def test_unreachable_state(self):
        self.assertEqual(solve(State(location='table_1', battery=0)), (None, None))

    def test_illegal_actions_are_rejected(self):
        for state, action in [(State(), Action('serve', '1')),
                              (State(location='kitchen', cooker_ok=False), Action('prepare', '1'))]:
            with self.assertRaises(ValueError):
                execute(state, action)

    def test_battery_and_tray_invariants(self):
        pending, seen = [State()], set()
        while pending:
            state = pending.pop()
            if state in seen:
                continue
            seen.add(state)
            self.assertTrue(0 <= state.battery <= 8)
            self.assertEqual(state.meals.count('carried'), int(state.carried >= 0))
            pending.extend(following for _, following, _ in successors(state))

    def test_world_load_contract(self):
        path = Path(__file__).resolve().parents[1] / 'worlds/restaurant.sdf'
        world = ET.parse(path).getroot().find('world')
        self.assertEqual(world.attrib['name'], 'robo_restaurant')
        self.assertEqual(len(world.findall('model')), 21)


if __name__ == '__main__':
    unittest.main()
