"""Check subprocess failure/timeout handling in the actual Nav2 notebook helper."""
import ast
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import unittest


class NotebookCommandTests(unittest.TestCase):
    def setUp(self):
        path = Path(__file__).parent / '11. Behavior Trees with Nav2 Helper Demo.ipynb'
        notebook = json.loads(path.read_text())
        source = next(''.join(cell['source']) for cell in notebook['cells']
                      if 'def ros_stdout' in ''.join(cell['source']))
        definition = next(node for node in ast.parse(source).body
                          if isinstance(node, ast.FunctionDef) and node.name == 'ros_stdout')
        namespace = dict(os=os, signal=signal, subprocess=subprocess,
                         ROS_DISTRO='not-installed-for-helper-test',
                         WORKSPACE=Path('/tmp/no-workshop-for-helper-test'))
        exec(compile(ast.Module(body=[definition], type_ignores=[]), str(path), 'exec'), namespace)
        self.run_command = namespace['ros_stdout']

    def test_success_returns_output_and_exit_code(self):
        result = self.run_command('printf ready', timeout_sec=2)
        self.assertEqual(result.returncode, 0)
        self.assertTrue(result.stdout.endswith('ready'))

    def test_checked_failure_raises(self):
        with self.assertRaises(subprocess.CalledProcessError) as error:
            self.run_command('exit 7', check=True, timeout_sec=2)
        self.assertEqual(error.exception.returncode, 7)

    def test_timeout_kills_pipeline_and_releases_output_pipe(self):
        started = time.monotonic()
        with self.assertRaises(subprocess.TimeoutExpired):
            self.run_command('sleep 10 | cat', timeout_sec=0.2)
        self.assertLess(time.monotonic() - started, 2)


if __name__ == '__main__':
    unittest.main()
