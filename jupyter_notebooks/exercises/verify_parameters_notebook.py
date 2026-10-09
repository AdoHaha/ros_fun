#!/usr/bin/env python3
"""Run exercise 8 in a fresh ROS kernel, optionally with the instructor answer.

Inside the sourced workshop container:
    python3 verify_parameters_notebook.py --repeat 2
    python3 verify_parameters_notebook.py --solutions --repeat 2

Uses isolated domain 97 by default and refuses another active graph. Outputs,
including executed answers, stay in /tmp. Close heavy Gazebo/RViz demonstrations
first: turtlesim's timed physics can lag under load.
"""
import argparse
import json
import os
from pathlib import Path

import nbformat
from nbclient import NotebookClient


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--solutions', action='store_true')
    parser.add_argument('--repeat', type=int, default=1)
    parser.add_argument('--domain-id', type=int, default=97)
    parser.add_argument('--output', type=Path, default=Path('/tmp/parameters-notebook-verification'))
    args = parser.parse_args()
    if args.repeat < 1:
        parser.error('--repeat must be positive')
    if not 0 <= args.domain_id <= 232:
        parser.error('--domain-id must be between 0 and 232')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    root = Path(__file__).resolve().parent
    args.output.mkdir(parents=True, exist_ok=True)
    profile = 'solved' if args.solutions else 'student'
    dest = args.output / f'{profile}.ipynb'
    report_path = args.output / f'{profile}-report.json'
    report = {'result': 'running', 'profile': profile, 'domain_id': args.domain_id,
              'runs_per_kernel': args.repeat, 'completed_runs': 0, 'motion': []}
    notebook = nbformat.read(root / '8. Parameters.ipynb', as_version=4)
    nbformat.validate(notebook)
    answers = json.loads((root / '.solutions/parameters_answers.json').read_text()) if args.solutions else {}
    for cell in notebook.cells:
        task = cell.metadata.get('parameters_task')
        if args.solutions and task in answers:
            cell.source = answers[task]
        if task == 'validator':
            cell.source += '\nassert walidacja_gotowa' if args.solutions else '\nassert not walidacja_gotowa'
        if cell.cell_type != 'code':
            continue
        if 'lab.start_turtlesim()' in cell.source:
            cell.source = cell.source.replace('lab.start_turtlesim()', '''
lab.spin_for(5)
existing = [(name, namespace) for name, namespace in node.get_node_names_and_namespaces()
            if not name.startswith('_ros2cli_daemon_')]
assert existing == [(node.get_name(), node.get_namespace())], (
    'Refusing to drive another active ROS graph; choose unused --domain-id', existing)
assert node.count_publishers('/turtle1/pose') == 0, 'Refusing an unowned turtlesim'
lab.start_turtlesim()
assert lab.process is not None, 'Verification requires an owned simulator'
''')
        if args.solutions and 'pozycja_przed =' in cell.source:
            cell.source = cell.source.replace('lab.watch_motion(ruch)', '''lab.watch_motion(ruch)
parameter_probe_commands = []
parameter_probe_subscription = node.create_subscription(
    Twist, '/turtle1/cmd_vel', parameter_probe_commands.append, 10)
lab.spin_for(0.5)''')
            cell.source += '''
import json
parameter_probe_distance = dystans_do_celu()
assert 0.2 < parameter_probe_distance <= 0.5, parameter_probe_distance
assert abs(stan['pose'].linear_velocity) < 0.01
assert abs(stan['pose'].angular_velocity) < 0.01
assert not dostawa_przyjeta()
assert parameter_probe_commands, 'No observed speed commands'
assert any(math.isclose(msg.linear.x, 0.8) for msg in parameter_probe_commands)
assert all(abs(msg.linear.x) <= 0.8 for msg in parameter_probe_commands)
assert parameter_probe_commands[-1].linear.x == parameter_probe_commands[-1].angular.z == 0
node.destroy_subscription(parameter_probe_subscription)
print('PARAMETERS_MOTION_PASS', json.dumps({
    'requested_speed': 1.5, 'observed_max_speed': max(abs(msg.linear.x) for msg in parameter_probe_commands),
    'distance_to_goal': parameter_probe_distance, 'final_velocity': stan['pose'].linear_velocity,
    'arrival_radius': node.get_parameter('arrival_radius').value,
    'accepted_after_radius_change': dostawa_przyjeta()}))
'''
    client = NotebookClient(notebook, timeout=90, kernel_name='python3',
                            resources={'metadata': {'path': str(root)}})
    try:
        with client.setup_kernel(cwd=str(root)):
            try:
                for run in range(args.repeat):
                    for index, cell in enumerate(notebook.cells):
                        client.execute_cell(cell, index, execution_count=client.code_cells_executed + 1)
                        if cell.cell_type == 'code':
                            for output in cell.get('outputs', []):
                                if output.output_type == 'stream':
                                    print(output.text, end='', flush=True)
                                    for line in output.text.splitlines():
                                        if line.startswith('PARAMETERS_MOTION_PASS '):
                                            report['motion'].append(json.loads(line.split(' ', 1)[1]))
                    report['completed_runs'] += 1
                    print('PARAMETERS_KERNEL_PASS', profile, run + 1, flush=True)
            finally:
                try:
                    cleanup = nbformat.v4.new_code_cell("if 'lab' in globals():\n    lab.close()")
                    notebook.cells.append(cleanup)
                    client.execute_cell(cleanup, len(notebook.cells) - 1)
                finally:
                    nbformat.write(notebook, dest)
        report['result'] = 'passed'
    except BaseException as error:
        report.update(result='failed', error=f'{type(error).__name__}: {error}')
        raise
    finally:
        report_path.write_text(json.dumps(report, indent=2) + '\n')
    print('PARAMETERS_NOTEBOOK_PASS', report, flush=True)


if __name__ == '__main__':
    main()
