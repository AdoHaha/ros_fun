#!/usr/bin/env python3
"""Execute BT student/reference notebooks in fresh kernels (inside ROS container).

Run from exercises/ with ROS and its overlay sourced:
    python3 verify_bt_notebooks.py
Add --ros to execute exercise 10's mock startup, real actions and cleanup too.
Executed notebooks and a JSON report are saved outside the source tree.
"""
import argparse
import json
from pathlib import Path

import nbformat
from nbclient import NotebookClient

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--ros', action='store_true')
parser.add_argument('--output', type=Path, default=Path('/tmp/bt-notebook-verification'))
args = parser.parse_args()
exercises = Path(__file__).resolve().parent
repo = exercises.parent.parent
args.output.mkdir(parents=True, exist_ok=True)
report = []
for path in sorted(exercises.rglob('*Behavior Trees*.ipynb')):
    if not path.name.startswith(('9.', '10.', '11.')):
        continue
    notebook = nbformat.read(path, as_version=4)
    if path.name.startswith('11.') and path.parent.name != '.solutions':
        # The first part exercises navigation decisions in the notebook without
        # a simulator. The live castle demo is verified separately with motion.
        boundary = next(index for index, cell in enumerate(notebook.cells)
                        if cell.cell_type == 'markdown' and '## Część B.' in cell.source)
        notebook.cells = notebook.cells[:boundary]
    # Real ROS cells are a separate, opt-in pass. Student stubs intentionally
    # cannot satisfy the ROS scenario until the exercises have been solved.
    if path.name.startswith('10.') and (not args.ros or path.parent.name != '.solutions'):
        notebook.cells = [cell for cell in notebook.cells if not (
            cell.cell_type == 'code' and any(marker in cell.source for marker in (
                'import os\nimport signal', 'from std_msgs.msg import Empty',
                "if 'ros_lab' in globals():")))]
    client = NotebookClient(notebook, timeout=90, kernel_name='python3',
                            resources={'metadata': {'path': str(path.parent)}})
    client.reset_execution_trackers()
    with client.setup_kernel(cwd=str(path.parent)):
        try:
            for index, cell in enumerate(notebook.cells):
                client.execute_cell(cell, index, execution_count=client.code_cells_executed + 1)
        except Exception:
            # Run the notebook's own cleanup while its kernel is still alive.
            for index, cell in enumerate(notebook.cells):
                if cell.cell_type == 'code' and cell.source.startswith("if 'ros_lab' in globals():"):
                    client.execute_cell(cell, index, execution_count=client.code_cells_executed + 1)
            nbformat.write(notebook, args.output / ('failed-' + path.name))
            raise
        client.set_widgets_metadata()
    result = notebook
    dest = args.output / ('solutions-' if path.parent.name == '.solutions' else 'student-')
    dest = dest.with_name(dest.name + path.name)
    nbformat.write(result, dest)
    cells = sum(cell.cell_type == 'code' for cell in result.cells)
    report.append({'notebook': str(path.relative_to(repo)), 'code_cells': cells, 'result': 'passed',
                   'ros': args.ros and path.parent.name == '.solutions' and path.name.startswith('10.')})
    print(f'PASS {path.name}: {cells} code cells', flush=True)
(args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
