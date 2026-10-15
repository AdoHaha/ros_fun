"""Async context lifecycle regression tests, using controllable service futures.

Run in the workshop ROS environment:
    python3 -m unittest discover -s exercises -p test_scan_context.py -v
"""

from concurrent.futures import Future
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

import py_trees
import rcl_interfaces.msg as msgs
import rcl_interfaces.srv as srvs

# Allow discovery from exercises/, the notebook students' working directory.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ros_fun_py_trees_ros_tutorials.behaviours import ScanContext
from ros_fun_py_trees_ros_tutorials import six_context_switching


class Client:
    def __init__(self):
        self.calls = []

    def wait_for_service(self, timeout_sec):
        return True

    def call_async(self, request):
        future = Future()
        self.calls.append((request, future))
        return future


class Node:
    def __init__(self):
        self.clients = {}
        self.errors = []

    def create_client(self, service_type, service_name):
        client = Client()
        self.clients[service_type] = client
        return client

    def get_logger(self):
        return self

    def error(self, message):
        self.errors.append(message)


def get_response(value=False):
    response = srvs.GetParameters.Response()
    response.values = [msgs.ParameterValue(
        type=msgs.ParameterType.PARAMETER_BOOL, bool_value=value
    )]
    return response


def set_response(successful=True):
    response = srvs.SetParameters.Response()
    response.results = [msgs.SetParametersResult(
        successful=successful, reason="" if successful else "rejected"
    )]
    return response


class ScanContextTests(unittest.TestCase):
    def setUp(self):
        self.node = Node()
        self.context = ScanContext("Scan Context")
        self.context.setup(node=self.node)
        self.get = self.node.clients[srvs.GetParameters]
        self.set = self.node.clients[srvs.SetParameters]

    def tick(self):
        self.context.tick_once()
        return self.context.status

    def finish_get(self, value=False):
        self.get.calls[-1][1].set_result(get_response(value))

    def finish_set(self, successful=True):
        self.set.calls[-1][1].set_result(set_response(successful))

    def requested_values(self):
        return [request.parameters[0].value.bool_value for request, _ in self.set.calls]

    def activate(self, original=False):
        self.tick()
        self.finish_get(original)
        self.finish_set()
        self.assertTrue(self.context.context_ready)

    def test_pending_get_does_not_block_or_repeat_requests(self):
        for _ in range(20):
            self.assertEqual(self.tick(), py_trees.common.Status.RUNNING)
        self.assertEqual(len(self.get.calls), 1)
        self.assertEqual(self.set.calls, [])
        self.assertFalse(self.context.context_ready)

    def test_pending_set_does_not_block_or_repeat_requests(self):
        self.tick()
        self.finish_get()
        for _ in range(20):
            self.assertEqual(self.tick(), py_trees.common.Status.RUNNING)
        self.assertEqual(self.requested_values(), [True])
        self.assertFalse(self.context.context_ready)

    def test_confirmed_context_stays_running_without_repeat_writes(self):
        self.activate()
        for _ in range(20):
            self.assertEqual(self.tick(), py_trees.common.Status.RUNNING)
        self.assertEqual(self.requested_values(), [True])

    def test_normal_exit_restores_without_further_ticks(self):
        self.activate()
        self.context.stop(py_trees.common.Status.SUCCESS)
        self.assertEqual(self.requested_values(), [True, False])
        self.assertTrue(self.context.cleanup_pending)
        self.finish_set()
        self.assertFalse(self.context.cleanup_pending)
        self.assertFalse(self.context.context_ready)

    def test_preemption_restores_true_original_value(self):
        self.activate(original=True)
        self.context.stop(py_trees.common.Status.INVALID)
        self.assertEqual(self.requested_values(), [True, True])
        self.finish_set()
        self.assertIsNone(self.context.cached_context)

    def test_interrupted_get_never_enables_context(self):
        self.tick()
        self.context.stop(py_trees.common.Status.INVALID)
        self.assertTrue(self.context.cleanup_pending)
        self.finish_get()
        self.assertEqual(self.set.calls, [])
        self.assertFalse(self.context.cleanup_pending)

    def test_interrupted_set_waits_for_response_before_restoring(self):
        self.tick()
        self.finish_get()
        self.context.stop(py_trees.common.Status.INVALID)
        self.assertEqual(self.requested_values(), [True])
        self.finish_set()
        self.assertEqual(self.requested_values(), [True, False])
        self.finish_set()
        self.assertFalse(self.context.cleanup_pending)

    def test_reentry_waits_for_restore_before_reading_original_again(self):
        self.activate()
        self.context.stop(py_trees.common.Status.INVALID)
        self.tick()
        self.assertEqual(len(self.get.calls), 1)
        self.assertFalse(self.context.context_ready)
        self.finish_set()
        self.assertEqual(len(self.get.calls), 2)
        self.finish_get()
        self.finish_set()
        self.assertEqual(self.requested_values(), [True, False, True])
        self.assertTrue(self.context.context_ready)

    def test_reentry_during_pending_set_reuses_original_value(self):
        self.tick()
        self.finish_get()
        self.context.stop(py_trees.common.Status.INVALID)
        self.tick()
        self.finish_set()
        self.assertTrue(self.context.context_ready)
        self.assertEqual(len(self.get.calls), 1)
        self.context.stop(py_trees.common.Status.INVALID)
        self.assertEqual(self.requested_values(), [True, False])

    def test_get_exception_reports_failure(self):
        self.tick()
        self.get.calls[-1][1].set_exception(RuntimeError("disconnected"))
        self.assertEqual(self.tick(), py_trees.common.Status.FAILURE)
        self.assertEqual(self.set.calls, [])
        self.assertTrue(self.node.errors)

    def test_empty_get_response_reports_failure(self):
        self.tick()
        self.get.calls[-1][1].set_result(srvs.GetParameters.Response())
        self.assertEqual(self.tick(), py_trees.common.Status.FAILURE)

    def test_wrong_parameter_type_reports_failure(self):
        self.tick()
        response = srvs.GetParameters.Response()
        response.values = [msgs.ParameterValue(type=msgs.ParameterType.PARAMETER_INTEGER)]
        self.get.calls[-1][1].set_result(response)
        self.assertEqual(self.tick(), py_trees.common.Status.FAILURE)

    def test_rejected_enable_reports_failure_and_restores_original(self):
        self.tick()
        self.finish_get()
        self.finish_set(successful=False)
        self.assertEqual(self.requested_values(), [True, False])
        self.assertEqual(self.tick(), py_trees.common.Status.FAILURE)
        self.finish_set()
        self.assertFalse(self.context.cleanup_pending)

    def test_exception_after_set_restores_possible_server_change(self):
        self.tick()
        self.finish_get()
        self.set.calls[-1][1].set_exception(RuntimeError("connection lost"))
        self.assertEqual(self.requested_values(), [True, False])
        self.assertEqual(self.tick(), py_trees.common.Status.FAILURE)
        self.finish_set()

    def test_restore_failure_is_logged_and_retried_before_reentry(self):
        self.activate()
        self.context.stop(py_trees.common.Status.INVALID)
        self.finish_set(successful=False)
        self.assertTrue(self.node.errors)
        self.tick()
        self.assertEqual(self.requested_values(), [True, False, False])
        self.assertEqual(len(self.get.calls), 1)
        self.finish_set()
        self.assertEqual(len(self.get.calls), 2)
        self.finish_get()
        self.finish_set()
        self.assertTrue(self.context.context_ready)

    def test_parallel_scan_completion_invalidates_context_and_restores(self):
        scan = py_trees.behaviours.Running("Scan")
        parallel = py_trees.composites.Parallel(
            "Scan with context",
            policy=py_trees.common.ParallelPolicy.SuccessOnSelected(
                children=[scan], synchronise=False
            ),
            children=[self.context, scan]
        )
        parallel.tick_once()
        self.finish_get()
        self.finish_set()
        scan.update = lambda: py_trees.common.Status.SUCCESS
        parallel.tick_once()
        self.assertEqual(parallel.status, py_trees.common.Status.SUCCESS)
        self.assertEqual(self.context.status, py_trees.common.Status.INVALID)
        self.assertEqual(self.requested_values(), [True, False])
        self.finish_set()

    def test_reactive_selector_preempts_running_context(self):
        emergency = py_trees.behaviours.Failure("Battery low?")
        root = py_trees.composites.Selector(
            "Priorities", memory=False, children=[emergency, self.context]
        )
        root.tick_once()
        self.finish_get()
        self.finish_set()
        emergency.update = lambda: py_trees.common.Status.RUNNING
        root.tick_once()
        self.assertEqual(self.context.status, py_trees.common.Status.INVALID)
        self.assertEqual(self.requested_values(), [True, False])
        self.finish_set()


class CountingRotate(py_trees.behaviour.Behaviour):
    """Record action activation without sending a real goal."""

    def __init__(self):
        super().__init__("Rotate")
        self.starts = 0
        self.result = py_trees.common.Status.RUNNING

    def initialise(self):
        self.starts += 1

    def update(self):
        return self.result


class TutorialSixContextGateTests(unittest.TestCase):
    def setUp(self):
        self.rotate = CountingRotate()
        with patch.object(six_context_switching.py_trees_ros.actions,
                          "ActionClient", return_value=self.rotate):
            root = six_context_switching.tutorial_create_root()
        self.scanning = next(node for node in root.iterate() if node.name == "Scanning")
        self.context = next(node for node in root.iterate() if isinstance(node, ScanContext))
        self.node = Node()
        self.context.setup(node=self.node)
        self.get = self.node.clients[srvs.GetParameters]
        self.set = self.node.clients[srvs.SetParameters]
        self.blue = next(node for node in root.iterate() if node.name == "Flash Blue")
        self.blue.publisher = Mock()

    def test_action_waits_for_get_and_set_confirmation(self):
        for _ in range(4):
            self.scanning.tick_once()
        self.assertEqual(self.rotate.starts, 0)
        self.get.calls[-1][1].set_result(get_response())
        for _ in range(4):
            self.scanning.tick_once()
        self.assertEqual(self.rotate.starts, 0)
        self.set.calls[-1][1].set_result(set_response())
        self.scanning.tick_once()
        self.assertEqual(self.rotate.starts, 1)
        self.assertEqual(self.scanning.status, py_trees.common.Status.RUNNING)
        self.assertEqual(self.blue.status, py_trees.common.Status.RUNNING)

    def test_context_failure_prevents_action_and_fails_parallel(self):
        self.scanning.tick_once()
        self.get.calls[-1][1].set_result(srvs.GetParameters.Response())
        self.scanning.tick_once()
        self.assertEqual(self.rotate.starts, 0)
        self.assertEqual(self.scanning.status, py_trees.common.Status.FAILURE)

    def test_rotation_completion_restores_context_and_clears_blue(self):
        self.scanning.tick_once()
        self.get.calls[-1][1].set_result(get_response())
        self.set.calls[-1][1].set_result(set_response())
        self.scanning.tick_once()
        self.rotate.result = py_trees.common.Status.SUCCESS
        self.scanning.tick_once()
        self.assertEqual(self.scanning.status, py_trees.common.Status.SUCCESS)
        self.assertEqual(self.context.status, py_trees.common.Status.INVALID)
        self.assertEqual(self.blue.status, py_trees.common.Status.INVALID)
        self.assertEqual(self.set.calls[-1][0].parameters[0].value.bool_value, False)
        self.assertEqual(self.blue.publisher.publish.call_args.args[0].data, "")

    def test_failed_enable_prevents_action(self):
        self.scanning.tick_once()
        self.get.calls[-1][1].set_result(get_response())
        self.set.calls[-1][1].set_result(set_response(successful=False))
        self.scanning.tick_once()
        self.assertEqual(self.scanning.status, py_trees.common.Status.FAILURE)
        self.assertEqual(self.rotate.starts, 0)


if __name__ == "__main__":
    unittest.main()
