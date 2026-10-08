"""Notebook diagrams generated from the actual py_trees objects."""
from html import escape
from pathlib import Path
import shutil
import tempfile

import py_trees
from IPython.display import HTML, display


def show_tree(root, title=None, blackboard=False):
    """Show an expandable DOT/SVG diagram and explain composite memory.

    SVG keeps its native size inside a scrollable frame, so a large application
    tree remains readable. DOT, SVG and PNG exports live outside student sources.
    """
    if shutil.which('dot') is None:
        print(py_trees.display.unicode_tree(root, show_status=True))
        print('Diagram SVG wymaga Graphviz; użyj obrazu warsztatowego Jazzy.')
        return
    folder = tempfile.mkdtemp(prefix='ros_fun_bt_diagram_')
    files = py_trees.display.render_dot_tree(
        root, target_directory=folder, with_blackboard_variables=blackboard,
    )
    svg = Path(files['svg']).read_text()
    svg = svg[svg.index('<svg'):]
    memories = ', '.join(
        f'{b.name}: memory={b.memory}' for b in root.iterate() if hasattr(b, 'memory')
    )
    blackboard_legend = ('<p>Blackboard: niebieskie strzałki oznaczają zapis, '
                         'zielone odczyt. To przepływ danych, a nie kolejność wykonywania.</p>'
                         if blackboard else '')
    display(HTML(
        '<details open><summary><strong>' + escape(title or root.name) + '</strong></summary>'
        '<p>Selector (ośmiokąt): pierwszy SUCCESS lub RUNNING; Sequence (prostokąt): '
        'dalej po SUCCESS; Parallel: dzieci w tym samym ticku. Kolejność dzieci od lewej.</p>'
        '<div style="overflow:auto; max-height:650px; border:1px solid #ddd">' + svg + '</div>'
        + blackboard_legend
        + '<p style="font-size:90%">' + escape(memories) + '</p></details>'
    ))
