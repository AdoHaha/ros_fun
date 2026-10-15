#
# License: BSD
#   https://github.com/splintered-reality/ros_fun_py_trees_ros_tutorials/raw/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Behaviours for the tutorials.
"""

##############################################################################
# Imports
##############################################################################

import py_trees
import py_trees_ros
import rcl_interfaces.msg as rcl_msgs
import rcl_interfaces.srv as rcl_srvs
import threading
import std_msgs.msg as std_msgs

##############################################################################
# Behaviours
##############################################################################


class FlashLedStrip(py_trees.behaviour.Behaviour):
    """
    This behaviour simply shoots a command off to the LEDStrip to flash
    a certain colour and returns :attr:`~py_trees.common.Status.RUNNING`.
    Note that this behaviour will never return with
    :attr:`~py_trees.common.Status.SUCCESS` but will send a clearing
    command to the LEDStrip if it is cancelled or interrupted by a higher
    priority behaviour.

    Publishers:
        * **/led_strip/command** (:class:`std_msgs.msg.String`)

          * colourised string command for the led strip ['red', 'green', 'blue']

    Args:
        name: name of the behaviour
        topic_name : name of the battery state topic
        colour: colour to flash ['red', 'green', blue']
    """
    def __init__(
            self,
            name: str,
            topic_name: str="/led_strip/command",
            colour: str="red"
    ):
        super(FlashLedStrip, self).__init__(name=name)
        self.topic_name = topic_name
        self.colour = colour

    def setup(self, **kwargs):
        """
        Setup the publisher which will stream commands to the mock robot.

        Args:
            **kwargs (:obj:`dict`): look for the 'node' object being passed down from the tree

        Raises:
            :class:`KeyError`: if a ros2 node isn't passed under the key 'node' in kwargs
        """
        self.logger.debug("{}.setup()".format(self.qualified_name))
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        self.publisher = self.node.create_publisher(
            msg_type=std_msgs.String,
            topic=self.topic_name,
            qos_profile=py_trees_ros.utilities.qos_profile_latched()
        )
        self.feedback_message = "publisher created"

    def update(self) -> py_trees.common.Status:
        """
        Annoy the led strip to keep firing every time it ticks over (the led strip will clear itself
        if no command is forthcoming within a certain period of time).
        This behaviour will only finish if it is terminated or priority interrupted from above.

        Returns:
            Always returns :attr:`~py_trees.common.Status.RUNNING`
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        self.publisher.publish(std_msgs.String(data=self.colour))
        self.feedback_message = "flashing {0}".format(self.colour)
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status):
        """
        Shoot off a clearing command to the led strip.

        Args:
            new_status: the behaviour is transitioning to this new status
        """
        self.logger.debug(
            "{}.terminate({})".format(
                self.qualified_name,
                "{}->{}".format(self.status, new_status) if self.status != new_status else "{}".format(new_status)
            )
        )
        self.publisher.publish(std_msgs.String(data=""))
        self.feedback_message = "cleared"


class ScanContext(py_trees.behaviour.Behaviour):
    """Enable the mock safety sensors while the scan branch is running.

    This context leaf stays RUNNING. A parallel can select the scan action as
    its completion condition and invalidate this leaf when the action finishes
    or a higher priority branch takes over. Service completion callbacks finish
    restoring the original parameter even after the tree stops ticking us.

    The ROS executor must keep spinning until restoration finishes. This is a
    demonstration of asynchronous cleanup, not a hardware safety controller.
    """

    def __init__(self, name):
        super().__init__(name=name)
        self.cached_context = None
        self.get_parameter_future = None
        self.set_parameter_future = None
        self._state = "idle"
        self._requested = False
        self._activation_failed = False
        # A MultiThreadedExecutor may deliver responses during a tree tick.
        self._lock = threading.RLock()

    @property
    def context_ready(self):
        """True once the service has confirmed the scan context is enabled."""
        with self._lock:
            return self._requested and self._state == "active"

    @property
    def cleanup_pending(self):
        """Whether a service response is still needed before cleanup finishes."""
        with self._lock:
            return not self._requested and self._state in ("getting", "setting", "restoring")

    def setup(self, **kwargs):
        """Create parameter clients; setup may wait, tree ticks never block."""
        try:
            self.node = kwargs['node']
        except KeyError as error:
            raise KeyError("didn't find 'node' in setup's kwargs [{}]".format(self.qualified_name)) from error
        self.parameter_clients = {
            'get_safety_sensors': self.node.create_client(
                rcl_srvs.GetParameters, '/safety_sensors/get_parameters'
            ),
            'set_safety_sensors': self.node.create_client(
                rcl_srvs.SetParameters, '/safety_sensors/set_parameters'
            )
        }
        for name, client in self.parameter_clients.items():
            if not client.wait_for_service(timeout_sec=3.0):
                raise RuntimeError("client timed out waiting for server [{}]".format(name))

    def initialise(self):
        """Request the context, preserving any cleanup still in progress."""
        with self._lock:
            self._requested = True
            self._activation_failed = False
            if self._state == "restore_failed":
                # Never cache an unrestored value as the next original value.
                self._send_set_parameter_request(self.cached_context, restoring=True)
            elif self._state in ("idle", "failed"):
                self._send_get_parameter_request()

    def update(self) -> py_trees.common.Status:
        """Maintain the context; response callbacks advance the service chain."""
        with self._lock:
            if self._activation_failed or self._state in ("failed", "restore_failed"):
                return py_trees.common.Status.FAILURE
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status):
        """Restore on any exit, including interruption during a service call.

        Do not cancel a set future: its request may already be executing on the
        server. Wait for its response and then send the restore request, keeping
        the two writes in order. A get completed after interruption does not
        change the parameter at all.
        """
        with self._lock:
            self._requested = False
            if self._state == "active":
                self._send_set_parameter_request(self.cached_context, restoring=True)

    def _fail(self, message, restoring=False):
        self._state = "restore_failed" if restoring else "failed"
        if not restoring:
            self._activation_failed = True
        self.feedback_message = message
        self.node.get_logger().error(message)

    def _send_get_parameter_request(self):
        self.cached_context = None
        self._state = "getting"
        self.feedback_message = "retrieving the safety sensors context"
        request = rcl_srvs.GetParameters.Request()
        request.names.append("enabled")
        try:
            self.get_parameter_future = self.parameter_clients['get_safety_sensors'].call_async(request)
            self.get_parameter_future.add_done_callback(self._get_parameter_done)
        except Exception as error:
            self._fail("failed to retrieve the safety sensors context: {}".format(error))

    def _get_parameter_done(self, future):
        with self._lock:
            if future is not self.get_parameter_future or self._state != "getting":
                return
            try:
                response = future.result()
                if response is None or len(response.values) != 1:
                    raise RuntimeError("expected exactly one parameter value")
                value = response.values[0]
                if value.type != rcl_msgs.ParameterType.PARAMETER_BOOL:
                    raise RuntimeError("expected a bool parameter")
                self.cached_context = value.bool_value
            except Exception as error:
                self._fail("failed to retrieve the safety sensors context: {}".format(error))
                return
            if self._requested:
                self._send_set_parameter_request(True)
            else:
                self.cached_context = None
                self._state = "idle"
                self.feedback_message = "context interrupted before any change"

    def _send_set_parameter_request(self, value: bool, restoring=False):
        self._state = "restoring" if restoring else "setting"
        self.feedback_message = "restoring the safety sensors context" if restoring else "enabling the safety sensors context"
        request = rcl_srvs.SetParameters.Request()
        parameter = rcl_msgs.Parameter()
        parameter.name = "enabled"
        parameter.value.type = rcl_msgs.ParameterType.PARAMETER_BOOL
        parameter.value.bool_value = value
        request.parameters.append(parameter)
        try:
            self.set_parameter_future = self.parameter_clients['set_safety_sensors'].call_async(request)
            self.set_parameter_future.add_done_callback(self._set_parameter_done)
        except Exception as error:
            self._fail("failed to {} the safety sensors context: {}".format(
                "restore" if restoring else "enable", error
            ), restoring=restoring)
            if not restoring:
                self._requested = False
                self._send_set_parameter_request(self.cached_context, restoring=True)

    def _set_parameter_done(self, future):
        with self._lock:
            if future is not self.set_parameter_future or self._state not in ("setting", "restoring"):
                return
            restoring = self._state == "restoring"
            try:
                response = future.result()
                if response is None or len(response.results) != 1:
                    raise RuntimeError("expected exactly one set result")
                if not response.results[0].successful:
                    raise RuntimeError(response.results[0].reason or "parameter write rejected")
            except Exception as error:
                self._fail("failed to {} the safety sensors context: {}".format(
                    "restore" if restoring else "enable", error
                ), restoring=restoring)
                if not restoring:
                    # An exceptional response does not prove the server never
                    # applied the write. Restore before allowing another scan.
                    self._requested = False
                    self._send_set_parameter_request(self.cached_context, restoring=True)
                return
            if restoring:
                self.cached_context = None
                self._state = "idle"
                self.feedback_message = "restored the safety sensors context"
                if self._requested:
                    self._send_get_parameter_request()
            elif self._requested:
                self._state = "active"
                self.feedback_message = "safety sensors enabled for scanning"
            else:
                self._send_set_parameter_request(self.cached_context, restoring=True)
