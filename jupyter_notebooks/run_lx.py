import os
import subprocess

def run_lxterminal(command):
    """start independent process in lxterminal, so output is visible and
    it is not killed by interrupt in jupyter notebook"""
    os.system('lxterminal --command="{} "&'.format(command))
