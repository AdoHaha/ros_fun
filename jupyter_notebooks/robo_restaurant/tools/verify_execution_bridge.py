"""Execute the headless bridge starter or separate instructor answer in Jupyter.

Inside the workshop container:
    python3 robo_restaurant/tools/verify_execution_bridge.py --repeat 2
    python3 robo_restaurant/tools/verify_execution_bridge.py --solutions --repeat 2
Executed copies are saved outside the source checkout by default.
"""
import argparse
import json
from pathlib import Path

import nbformat
from nbclient import NotebookClient


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--solutions', action='store_true')
    parser.add_argument('--repeat', type=int, default=1)
    parser.add_argument('--output', type=Path,
                        default=Path('/tmp/planning-execution-verification'))
    args = parser.parse_args()
    if args.repeat < 1:
        parser.error('--repeat must be positive')
    exercises = Path(__file__).resolve().parents[2] / 'exercises'
    notebook = nbformat.read(exercises / '15. Planning and Execution - Bridge.ipynb', as_version=4)
    nbformat.validate(notebook)
    if args.solutions:
        answers = json.loads((exercises / '.solutions/planning_execution_answers.json').read_text())
        for cell in notebook.cells:
            task = cell.metadata.get('planning_execution_task')
            if task in answers:
                cell.source = answers[task]
    client = NotebookClient(notebook, timeout=30, kernel_name='python3',
                            resources={'metadata': {'path': str(exercises)}})
    args.output.mkdir(parents=True, exist_ok=True)
    mode = 'solved' if args.solutions else 'student'
    destination = args.output / f'{mode}.ipynb'
    with client.setup_kernel(cwd=str(exercises)):
        try:
            for run in range(args.repeat):
                for index, cell in enumerate(notebook.cells):
                    client.execute_cell(cell, index, execution_count=client.code_cells_executed + 1)
                output = ''.join(item.get('text', '') for cell in notebook.cells
                                 if cell.cell_type == 'code' for item in cell.outputs)
                assert ('PASS: retry, replan, blocked' in output) == args.solutions
                assert ('TODO: implement recovery_policy' in output) != args.solutions
                print(f'PASS {mode}: kernel run {run + 1}', flush=True)
        finally:
            nbformat.write(notebook, destination)
    print(f'Executed notebook: {destination}')


if __name__ == '__main__':
    main()
