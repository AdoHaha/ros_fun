import os
import os
import json
import nbformat as nbf
from natsort import natsorted


def add_links_to_notebooks(directory="."):
    # Get a list of all .ipynb files in the directory
    notebooks = [f for f in os.listdir(directory) if f.endswith(".ipynb")]

    # Natural sort the notebooks to handle numbered files correctly
    notebooks = natsorted(notebooks)

    for i in range(len(notebooks) - 1):
        # Open the notebook
        with open(notebooks[i]) as f:
            nb = nbf.read(f, as_version=4)

        # Get the name of the next notebook without the .ipynb extension
        next_notebook_title = os.path.splitext(notebooks[i+1])[0]

        # Create a URL-safe version of the next notebook's filename
        next_notebook_filename = notebooks[i+1].replace(' ', '%20')

        # Create a new markdown cell with a link to the next notebook
        new_cell = nbf.v4.new_markdown_cell(source=f"[Next exercise: {next_notebook_title}]({next_notebook_filename})")

        # Add the new cell to the end of the notebook
        nb.cells.append(new_cell)

        # Write the updated notebook back to the file
        with open(notebooks[i], "w") as f:
            nbf.write(nb, f)

if __name__ == "__main__":
    add_links_to_notebooks()
