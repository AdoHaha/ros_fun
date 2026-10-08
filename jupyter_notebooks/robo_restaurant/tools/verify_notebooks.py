"""Execute student notebooks without filling their answer cells."""
import argparse
from pathlib import Path
import nbformat
from nbclient import NotebookClient
ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    paths = sorted((ROOT / 'exercises').glob('*Robo Restaurant*.ipynb'))
    assert len(paths) == 3, 'Expected all three student notebooks'
    for path in paths:
        notebook = nbformat.read(path, as_version=4)
        nbformat.validate(notebook)
        NotebookClient(notebook, timeout=60, kernel_name='python3',
                       resources={'metadata': {'path': str(path.parent)}}).execute()
        output = ''.join(o.get('text', '') for c in notebook.cells if c.cell_type == 'code'
                         for o in c.outputs)
        assert 'TODO: implement' in output, 'Unanswered exercise must remain marked TODO'
        nbformat.write(notebook, args.output / path.name)
        print(f'PASS starter execution: {path.name}')


if __name__ == '__main__':
    main()
