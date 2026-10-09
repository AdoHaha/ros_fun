"""Exercise the process lifecycle that now replaces notebook shell pipelines."""
from pathlib import Path
import subprocess
import tempfile
import time
import unittest

from navigation_lab import NavigationLab, stop_owned_process


class OwnedProcessTests(unittest.TestCase):
    def make_lab(self, output):
        lab = NavigationLab.__new__(NavigationLab)
        lab.output = Path(output)
        lab.processes = []
        return lab

    def test_launch_records_owned_process_and_output(self):
        with tempfile.TemporaryDirectory() as output:
            lab = self.make_lab(output)
            lab._launch('probe', ['python3', '-c', 'print("ready")'])
            name, process, log = lab.processes[0]
            try:
                self.assertEqual(process.wait(timeout=3), 0)
                self.assertEqual(name, 'probe')
                self.assertEqual(log.read_text().strip(), 'ready')
            finally:
                stop_owned_process(process)

    def test_early_launch_failure_reports_log(self):
        with tempfile.TemporaryDirectory() as output:
            lab = self.make_lab(output)
            lab._launch('probe', ['python3', '-c', 'raise SystemExit(7)'])
            process = lab.processes[0][1]
            try:
                self.assertEqual(process.wait(timeout=3), 7)
                with self.assertRaisesRegex(RuntimeError, 'probe.*probe.log'):
                    lab._check_processes()
            finally:
                stop_owned_process(process)

    def test_owned_pipeline_children_release_output_on_cleanup(self):
        # A live pipe remains open if only its launcher is stopped.
        process = subprocess.Popen(['bash', '-c', 'sleep 30 | cat'],
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                   start_new_session=True)
        try:
            time.sleep(0.1)
            stop_owned_process(process)
            process.communicate(timeout=2)
            self.assertIsNotNone(process.returncode)
        finally:
            stop_owned_process(process)
            process.stdout.close()
            process.stderr.close()


if __name__ == '__main__':
    unittest.main()
