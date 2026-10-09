"""Bounded terminal calls for the parameters mission; no background spinner.

Parameter declarations and validation remain visible in the lesson notebook.
"""
import math
import os
import signal
import subprocess
import time


def run_parameter_cli(lab, arguments, timeout=12.0):
    """Run ros2 param while the notebook executor serves the CLI request.

    Returns CompletedProcess, including rejection output; a nonzero exit status
    is a CLI error. Killing an owned CLI process never reverses a server update.
    """
    if not arguments:
        raise ValueError('Podaj polecenie list, get lub set.')
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError('Limit czasu musi być dodatni i skończony.')
    # Fresh discovery avoids a daemon retaining the previous notebook node.
    command = ['ros2', 'param', arguments[0], '--no-daemon', '--spin-time', '2',
               *arguments[1:]]
    process = subprocess.Popen(command, stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE, text=True,
                               start_new_session=True)
    try:
        deadline = time.monotonic() + timeout
        while process.poll() is None:
            if time.monotonic() >= deadline:
                raise TimeoutError('CLI nie odpowiedziało. Sprawdź nazwę węzła i spinowanie.')
            lab.spin_for(0.05)
        output, error = process.communicate(timeout=1)
        result = subprocess.CompletedProcess(command, process.returncode, output, error)
        if result.returncode:
            raise RuntimeError(f'CLI: {result.stderr or result.stdout}')
        return result
    finally:
        # A ros2 launcher may exit before its child; clean up the owned group.
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            try:
                os.killpg(process.pid, sig)
            except ProcessLookupError:
                break
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                continue
        process.stdout.close()
        process.stderr.close()
