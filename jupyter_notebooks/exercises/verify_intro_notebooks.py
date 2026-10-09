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
            if args.solutions and cell.metadata.get('intro_task') == 'square':
                cell.source = ('from intro_live_checks import begin_square, check_square\n'
                               'square_probe, square_start = begin_square(lab)\n'
                               + cell.source + '\ncheck_square(square_probe, square_start)')
            if 'intro-widget' in cell.metadata.get('tags', []):
                cell.source += ('\nfrom intro_live_checks import check_panel\n'
                                'check_panel(lab, przyciski)')
                if args.solutions and path.name.startswith('4.'):
                    cell.source += ('\nfrom intro_live_checks import drive_checkpoints\n'
                                    'drive_checkpoints(lab, ruch, cele, odwiedzone, pokaz_stan)\n'
                                    'assert "<svg" in status.value')
        client = NotebookClient(notebook, timeout=90, kernel_name='python3',
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
