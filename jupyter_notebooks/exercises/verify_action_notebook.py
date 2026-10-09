#!/usr/bin/env python3
"""Execute student and solved exercise 7 with real turtlesim, including cancellation.

Run as the Ubuntu desktop/Jupyter user in the sourced workshop container.
Optional Nav2 is verified with the navigation lesson rather than launched here.
"""
import argparse
import json
import os
from pathlib import Path
import time

import nbformat
from nbclient import NotebookClient


def wait_cli_output(lab, command, expected, timeout=15):
    """Give the CLI's newly started discovery participant time to see actions."""
    from verify_intro_cli import run_cli
    deadline = time.monotonic() + timeout
    output = ''
    while time.monotonic() < deadline:
        output = run_cli(command, timeout=max(0.1, deadline - time.monotonic()))
        if expected in output:
            return output
        lab.spin_for(0.2)
    raise AssertionError((command, expected, output))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repeat', type=int, default=2)
    parser.add_argument('--domain-id', type=int, default=96)
    parser.add_argument('--output', type=Path, default=Path('/tmp/action-notebook-verification'))
    args = parser.parse_args()
    if args.repeat < 1 or not 0 <= args.domain_id <= 232:
        parser.error('Use a positive --repeat and a --domain-id between 0 and 232')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    exercises = Path(__file__).resolve().parent
    args.output.mkdir(parents=True, exist_ok=True)
    answers = json.loads((exercises / '.solutions/action_answers.json').read_text())
    report = []
    for solved in (False, True):
        notebook = nbformat.read(exercises / '7. ROS Action.ipynb', as_version=4)
        notebook.cells = [cell for cell in notebook.cells
                          if 'action-optional' not in cell.metadata.get('tags', [])]
        for cell in notebook.cells:
            task = cell.metadata.get('action_task')
            if solved and task:
                cell.source = answers[task] + '\nassert kompas_gotowy'
            tags = cell.metadata.get('tags', [])
            if 'action-demo-result' in tags:
                cell.source += ('\nassert wynik_demo.status == GoalStatus.STATUS_SUCCEEDED'
                                '\nassert len(proba_demo["feedback"]) > 0'
                                '\nassert abs(pozycja["pose"].theta - math.pi / 2) < 0.05')
                # CLI commands run while the real server is alive. A timeout
                # applies to each owned CLI process, including send_goal.
                cell.source += (
                    '\nfrom verify_intro_cli import run_cli'
                    '\nfrom verify_action_notebook import wait_cli_output'
                    '\nwait_cli_output(lab, ["ros2", "action", "list", "-t"], "turtlesim/action/RotateAbsolute")'
                    '\nwait_cli_output(lab, ["ros2", "action", "info", "/turtle1/rotate_absolute"], "Action servers: 1")'
                    '\nrun_cli(["ros2", "interface", "show", "turtlesim/action/RotateAbsolute"], "float32 remaining")')
            if solved and 'action-compass-run' in tags:
                cell.source += ('\nassert len(proby_akcji) == 5'
                                '\nassert all(p["result"].status == GoalStatus.STATUS_SUCCEEDED for p in proby_akcji)'
                                '\nassert abs(pozycja["pose"].theta) < 0.05')
            if 'action-cancellation' in tags:
                cell.source += ('\nassert cancel_reply.return_code == 0'
                                '\nassert len(cancel_reply.goals_canceling) == 1'
                                '\nassert wynik_stop.status == GoalStatus.STATUS_CANCELED'
                                '\nassert 0.1 < kat_przed_anulowaniem < 2.5'
                                '\nassert abs(pozycja["pose"].theta - kat_po_anulowaniu) < 0.03'
                                '\nassert abs(pozycja["pose"].angular_velocity) < 0.01')
                cell.source += (
                    '\nrun_cli(["timeout", "15s", "ros2", "action", "send_goal", "/turtle1/rotate_absolute",'
                    ' "turtlesim/action/RotateAbsolute", "{theta: 0.0}", "--feedback", "--timeout", "10"],'
                    ' "SUCCEEDED", timeout=17)')
        nbformat.validate(notebook)
        client = NotebookClient(notebook, timeout=90, kernel_name='python3',
                                resources={'metadata': {'path': str(exercises)}})
        dest = args.output / (('solved-' if solved else 'student-') + '7. ROS Action.ipynb')
        with client.setup_kernel(cwd=str(exercises)):
            try:
                for _ in range(args.repeat):
                    for index, cell in enumerate(notebook.cells):
                        client.execute_cell(cell, index,
                                            execution_count=client.code_cells_executed + 1)
            finally:
                cleanup = nbformat.v4.new_code_cell(
                    "from action_workshop import close_action_lab\nclose_action_lab(globals())")
                notebook.cells.append(cleanup)
                try:
                    client.execute_cell(cleanup, len(notebook.cells) - 1)
                finally:
                    nbformat.write(notebook, dest)
        report.append({'notebook': '7. ROS Action.ipynb', 'solutions': solved,
                       'runs_per_kernel': args.repeat, 'result': 'passed'})
        print(f'PASS exercise 7: {"solved" if solved else "student"}, {args.repeat} runs', flush=True)
    (args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()
