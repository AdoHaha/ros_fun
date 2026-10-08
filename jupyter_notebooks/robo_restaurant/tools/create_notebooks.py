"""Regenerate unexecuted student and instructor notebooks with nbformat."""
from pathlib import Path
import nbformat as nbf

ROOT = Path(__file__).resolve().parents[1]
BOOT = '''from pathlib import Path
import sys
# Works from the student folder, solutions folder, or notebook server root.
base = Path.cwd().resolve()
for candidate in (base, *base.parents, base / 'jupyter_notebooks'):
    if (candidate / 'robo_restaurant' / 'planning.py').exists():
        sys.path.insert(0, str(candidate))
        break
else:
    raise RuntimeError('Open this notebook inside the repository')
from dataclasses import replace
from robo_restaurant.planning import State, Action, solve, execute, successors, goal
'''

def write(name, title, intro, demo, task, signature, checks, answer, explanation):
    cells = [nbf.v4.new_markdown_cell(f'# {title}\n\n{intro}\n\n'
        'These are symbolic planning exercises. Gazebo is a separate static scene; '
        'these cells do not command a robot. Run cells in order with Python 3.'),
        nbf.v4.new_code_cell(BOOT), nbf.v4.new_markdown_cell('## Demonstration'),
        nbf.v4.new_code_cell(demo), nbf.v4.new_markdown_cell('## Your task\n\n'+task),
        nbf.v4.new_code_cell(signature+'\n    # TODO: replace this body.\n    raise NotImplementedError\n'),
        nbf.v4.new_markdown_cell('## Check your answer\n\n'
            'The starter runs without errors but reports an unfinished task. '
            'After implementing it, this cell must report PASS.'),
        nbf.v4.new_code_cell('try:\n'+ '\n'.join('    '+line for line in checks.splitlines())+
            '\nexcept NotImplementedError:\n    print("TODO: implement the exercise; this is not a pass")\nelse:\n    print("PASS")'),
        nbf.v4.new_markdown_cell('## Discussion\n\n'+explanation)]
    metadata={'kernelspec': {'display_name':'Python 3','language':'python','name':'python3'},
              'language_info':{'name':'python'}}
    nbf.write(nbf.v4.new_notebook(cells=cells,metadata=metadata), ROOT / f'{name}.ipynb')
    cells[4]=nbf.v4.new_markdown_cell('## Reference solution\n\n'+task)
    cells[5]=nbf.v4.new_code_cell(answer)
    cells[6]=nbf.v4.new_markdown_cell('## Verification')
    cells[7]=nbf.v4.new_code_cell(checks+'\nprint("PASS")')
    scenario = {'01_normal_service': 'normal', '02_low_battery': 'low_battery',
                '03_cooker_failure': 'cooker_failure'}[name]
    cells.append(nbf.v4.new_markdown_cell('## Verify the solution in Gazebo\n\n'
        'Set `ROBO_RESTAURANT_GAZEBO=1` before launching Jupyter for a private headless test, '
        'or set it to `live` to update a restaurant world you already opened with `gz sim -r`. '
        'The waiter is a discrete visual proxy; this does not test autonomous driving.'))
    cells.append(nbf.v4.new_code_cell(
        'import os\n'
        'mode = os.environ.get("ROBO_RESTAURANT_GAZEBO", "off")\n'
        'if mode in ("1", "live"):\n'
        '    from contextlib import nullcontext\n'
        '    from robo_restaurant.gazebo import isolated_restaurant, GazeboRestaurant\n'
        '    from robo_restaurant.solutions.answers import run_world_solution\n'
        '    session = isolated_restaurant() if mode == "1" else nullcontext(GazeboRestaurant())\n'
        '    with session as world:\n'
        f'        result = run_world_solution("{scenario}", world)\n'
        '    assert goal(result["state"])\n'
        '    print("PASS Gazebo:", result)\n'
        'else:\n'
        '    print("Gazebo check skipped; set ROBO_RESTAURANT_GAZEBO=1 to enable")'))
    nbf.write(nbf.v4.new_notebook(cells=cells,metadata=metadata), ROOT/'solutions'/f'{name}_solution.ipynb')

write('01_normal_service', '1. Plan normal restaurant service',
      'Goals: identify preconditions and effects, inspect a minimum-cost plan, and validate execution. '
      'One tray holds one meal. Only travel drains energy; total action cost is not elapsed time.',
      '''initial = State()
plan, cost = solve(initial)
print('Optimal cost:', cost)
for action in plan:
    print(action)
assert cost == 25''',
      'Implement `audit_plan(initial, plan)` returning `(final_state, total_cost)`. '
      'Use legal transitions from `successors`; reject an illegal action with `ValueError`. '
      'Do not call `solve`: you are validating a supplied plan. A partial plan may be legal without reaching the goal.',
      'def audit_plan(initial, plan):',
      '''final, actual = audit_plan(initial, plan)
assert goal(final) and actual == 25
assert audit_plan(initial, []) == (initial, 0)
try:
    audit_plan(initial, [Action('serve', '1')])
except ValueError:
    pass
else:
    raise AssertionError('Illegal serving must be rejected')''',
      'from robo_restaurant.solutions.answers import audit_plan',
      'Why does the plan charge even with an initially full battery? '
      'List the preconditions of pickup and serving. Why is a valid partial plan different from a completed goal?')

write('02_low_battery', '2. Plan with limited battery',
      'Goals: distinguish a resource constraint from a cost, compare charging requirements, '
      'and report a genuinely unreachable state.',
      '''initial = State(battery=2)
plan, cost = solve(initial)
print('Low-battery cost:', cost)
print('Charging actions:', sum(a.name == 'charge' for a in plan))
assert cost == 28
stranded = State(location='table_1', battery=0)
assert solve(stranded) == (None, None)
print('A waiter stranded away from the dock cannot recover in this model.')''',
      'Implement `energy_report(initial)` returning a dictionary with `feasible`, `cost`, '
      'and `charges`. Use `solve`; for an unreachable state return `False`, `None`, `None`. '
      'Compare initial batteries 0, 2 and 8. Do not assume every low-battery state can reach a charger.',
      'def energy_report(initial):',
      '''assert energy_report(State(battery=2)) == {'feasible': True, 'cost': 28, 'charges': 2}
assert energy_report(State()) == {'feasible': True, 'cost': 25, 'charges': 1}
assert energy_report(State(battery=0)) == {'feasible': True, 'cost': 28, 'charges': 2}
assert energy_report(stranded) == {'feasible': False, 'cost': None, 'charges': None}
for battery in range(9):
    print(battery, energy_report(State(battery=battery)))''',
      'from robo_restaurant.solutions.answers import energy_report',
      'Why can battery 0 at the dock be feasible while battery 0 at a table is not? '
      'Propose a rescue action with explicit preconditions, effects and cost. '
      'Optional extension: parameterize capacity instead of changing isolated numeric literals.')

write('03_cooker_failure', '3. Observe a failure and replan',
      'Goals: detect a stale action, preserve the observed physical state and obtain a recovery plan.',
      '''initial = State()
stale_plan, _ = solve(initial)
observed = initial
while stale_plan[0].name != 'prepare':
    observed = execute(observed, stale_plan.pop(0))
observed = replace(observed, cooker_ok=False)
print('Cooker failure observed at:', observed)
try:
    execute(observed, stale_plan[0])
except ValueError as error:
    print('Expected stale-action rejection:', error)
else:
    raise AssertionError('Preparation on a broken cooker must fail')''',
      'Implement `recover(observed)` returning `status`, `plan`, and `cost`. '
      'Solve from the observed state, not from the original initial state. '
      'For a feasible plan return status `ready`; otherwise return `blocked` with plan/cost `None`. '
      'Replay a feasible plan to verify goal satisfaction and its cost before returning it.',
      'def recover(observed):',
      '''result = recover(observed)
assert result['status'] == 'ready' and result['cost'] == 28
assert result['plan'][0] == Action('repair')
state = observed
for action in result['plan']:
    state = execute(state, action)
assert goal(state)
assert recover(State(location='table_1', battery=0, cooker_ok=False)) == {
    'status': 'blocked', 'plan': None, 'cost': None}
print('Total executed cost: travel before failure 2 + recovery 28 = 30')''',
      'from robo_restaurant.solutions.answers import recover',
      'Which preparation precondition changed? Why must recovery preserve battery and location? '
      'The event is external, not a planner action. What changes when failure can happen during an action?')

if __name__ == '__main__':
    print('Generated three student notebooks and three instructor notebooks.')
