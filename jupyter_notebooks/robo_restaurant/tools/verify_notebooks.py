"""Execute starters, student tasks filled with solutions, and instructor notebooks."""
import argparse
from pathlib import Path
import nbformat
from nbclient import NotebookClient
ROOT = Path(__file__).resolve().parents[1]
STUDENT_NAMES = {'01_normal_service': '12. Robo Restaurant - Normal Service', '02_low_battery': '13. Robo Restaurant - Low Battery', '03_cooker_failure': '14. Robo Restaurant - Cooker Failure'}


def execute(path, output, fill=False):
    notebook = nbformat.read(path, as_version=4)
    nbformat.validate(notebook)
    if fill:
        solution = nbformat.read(ROOT / 'solutions' / f'{next(k for k, v in STUDENT_NAMES.items() if v == path.stem)}_solution.ipynb', as_version=4)
        notebook.cells[5].source = solution.cells[5].source
    NotebookClient(notebook, timeout=60, kernel_name='python3',
                   resources={'metadata': {'path': str(path.parent)}}).execute()
    text = ''.join(o.get('text', '') for c in notebook.cells if c.cell_type == 'code' for o in c.outputs)
    expected = 'PASS' if fill or path.parent.name == 'solutions' else 'TODO: implement'
    assert expected in text, f'{path}: missing {expected}'
    if expected == 'PASS':
        assert 'TODO: implement' not in text
    label = path.stem + ('_filled' if fill else '')
    nbformat.write(notebook, output / f'{label}.ipynb')
    print(f'PASS execution: {label}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    for path in sorted((ROOT.parent / 'exercises').glob('*Robo Restaurant*.ipynb')):
        execute(path, args.output)
        execute(path, args.output, fill=True)
    for path in sorted((ROOT / 'solutions').glob('*.ipynb')):
        execute(path, args.output)


if __name__ == '__main__':
    main()
