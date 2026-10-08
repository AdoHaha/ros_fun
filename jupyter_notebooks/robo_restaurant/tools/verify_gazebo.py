"""Run all three instructor solutions against a private headless Gazebo world."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from robo_restaurant.gazebo import WORLD, command, isolated_restaurant
from robo_restaurant.solutions.answers import run_world_solution


def main():
    print(command(['sdf', '-k', str(WORLD)]).strip())
    expected_models = {m.attrib['name'] for m in ET.parse(WORLD).getroot().find('world').findall('model')}
    with isolated_restaurant() as world:
        assert set(world.scene()) == expected_models
        stats = command(['topic', '-e', '-t', '/world/robo_restaurant/stats', '-n', '2', '--json-output'], world.env)
        assert 'iterations' in stats and 'simTime' in stats, stats
        for scenario, cost in [('normal', 25), ('low_battery', 28), ('cooker_failure', 30)]:
            result = run_world_solution(scenario, world)
            assert result['cost'] == cost, result
            assert result['failure_injected'] == (scenario == 'cooker_failure')
            print(f'PASS Gazebo {scenario}: cost={cost}, {result["steps"]} actions; scene verified after every transition')
    print(f'PASS: {len(expected_models)} models loaded; simulation statistics published')


if __name__ == '__main__':
    main()
