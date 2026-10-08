#!/usr/bin/env python3
"""Run exercises 1–5 in fresh ROS kernels, with optional instructor answers.

Inside the sourced workshop container:
    python3 verify_intro_notebooks.py
    python3 verify_intro_notebooks.py --solutions
Outputs, including solved copies, are written only to /tmp by default.
GUI/CLI interaction and optional Gazebo cells are excluded; widgets are constructed.
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
    parser.add_argument('--repeat', type=int, default=1, help='Full runs per kernel (test reruns)')
    parser.add_argument('--output', type=Path, default=Path('/tmp/intro-notebook-verification'))
    args = parser.parse_args()
    if args.repeat < 1:
        parser.error('--repeat must be positive')
    exercises = Path(__file__).resolve().parent
    args.output.mkdir(parents=True, exist_ok=True)
    # A separate ROS graph avoids using or moving a participant's simulator.
    os.environ['ROS_DOMAIN_ID'] = '87'
    answers = json.loads((exercises / '.solutions/intro_answers.json').read_text()) if args.solutions else {}
    report = []
    for path in sorted(exercises.glob('[1-5]. *.ipynb')):
        notebook = nbformat.read(path, as_version=4)
        nbformat.validate(notebook)
        notebook.cells = [cell for cell in notebook.cells if not (
            cell.cell_type == 'code' and set(cell.metadata.get('tags', [])).intersection(
                {'intro-interactive', 'intro-optional'}))]
        for cell in notebook.cells:
            if cell.metadata.get('intro_task') in answers:
                cell.source = answers[cell.metadata.intro_task]
        if args.solutions:
            for cell in notebook.cells:
                task = cell.metadata.get('intro_task')
                if task:
                    cell.source += '\n' + {
                        'numbered_messages': "assert wyniki == ['Paczka 1', 'Paczka 2', 'Paczka 3']",
                        'square': 'assert bok_predkosc > 0 and skret_predkosc > 0',
                        'distance': 'assert odleglosc_gotowa',
                        'delivery': 'assert dostawa_gotowa',
                    }[task]
                if 'intro-finale' in cell.metadata.get('tags', []):
                    cell.source += ('\nassert not odpowiedz_przed.success\n'
                                    'assert odpowiedz_po.success\n'
                                    'assert not odpowiedz_druga.success\n'
                                    'assert bilet["numer"] == 1')
        for cell in notebook.cells:
            if 'intro-widget' in cell.metadata.get('tags', []):
                # Trigger the actual widget callback and observe ROS motion.
                cell.source += '''
from turtlesim.msg import Pose as ProbePose
probe = {'pose': None}
probe_subscription = node.create_subscription(ProbePose, '/turtle1/pose', lambda msg: probe.update(pose=msg), 10)
lab.wait_for(lambda: probe['pose'] is not None, description='widget motion probe')
before_click = probe['pose']
przyciski[1][0].click()  # actual Forward button callback
lab.spin_for(0.1)
after_click = probe['pose']
assert ((after_click.x-before_click.x)**2 + (after_click.y-before_click.y)**2)**0.5 > 0.1
assert abs(after_click.linear_velocity) < 0.01
node.destroy_subscription(probe_subscription)
'''
                if args.solutions and path.name.startswith('4.'):
                    cell.source += '''
from turtlesim.srv import TeleportAbsolute as ProbeTeleport
for target_x, target_y in cele.values():
    lab.call(ProbeTeleport, '/turtle1/teleport_absolute', ProbeTeleport.Request(x=target_x, y=target_y, theta=0.0))
    lab.spin_for(0.2)
assert odwiedzone == {'A', 'B', 'C'}
lab.spin_for(0.2)
assert len(odwiedzone) == 3
pokaz_stan()
assert '<svg' in status.value
'''
        client = NotebookClient(notebook, timeout=60, kernel_name='python3',
                                resources={'metadata': {'path': str(exercises)}})
        dest = args.output / (('solved-' if args.solutions else 'student-') + path.name)
        with client.setup_kernel(cwd=str(exercises)):
            try:
                for _ in range(args.repeat):
                    for index, cell in enumerate(notebook.cells):
                        client.execute_cell(cell, index, execution_count=client.code_cells_executed + 1)
            finally:
                # Even a failing task must stop owned motion and GUI processes.
                final_cleanup = nbformat.v4.new_code_cell("if 'lab' in globals():\n    lab.close()")
                notebook.cells.append(final_cleanup)
                client.execute_cell(final_cleanup, len(notebook.cells) - 1)
                client.set_widgets_metadata()
                nbformat.write(notebook, dest)
        cells = sum(cell.cell_type == 'code' for cell in notebook.cells)
        report.append({'notebook': path.name, 'code_cells': cells, 'result': 'passed',
                       'solutions': args.solutions, 'runs_per_kernel': args.repeat})
        print(f'PASS {path.name}: {cells} code cells', flush=True)
    (args.output / ('solutions-report.json' if args.solutions else 'student-report.json')).write_text(
        json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()
