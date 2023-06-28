from std_srvs.srv import Empty
from rclpy.node import Node

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import ParameterValue, ParameterType, Parameter
import rclpy

def clear_simulate(node: Node):
    # Create service clients
    pause_client = node.create_client(Empty, "/pause_physics")
    reset_client = node.create_client(Empty, "/reset_simulation")
    unpause_client = node.create_client(Empty, "/unpause_physics")

    # Create requests
    pause_req = Empty.Request()
    reset_req = Empty.Request()
    unpause_req = Empty.Request()

    # Check service availability and call services synchronously
    if pause_client.wait_for_service(timeout_sec=10):
        pause_future = pause_client.call_async(pause_req)
        rclpy.spin_until_future_complete(node,pause_future)
    else:
        print('Failed to connect to the pause service')
        return
    if reset_client.wait_for_service(timeout_sec=10):
        reset_future = reset_client.call_async(reset_req)
        rclpy.spin_until_future_complete(node,reset_future)
    else:
        print('Failed to connect to the reset service')
        return
    if unpause_client.wait_for_service(timeout_sec=10):
        unpause_future = unpause_client.call_async(unpause_req)
        rclpy.spin_until_future_complete(node,unpause_future)
    else:
        print('Failed to connect to the unpause service')
        return
    # Confirm success of service calls
    if pause_future.result() is not None and reset_future.result() is not None and unpause_future.result() is not None:
        print('Simulation paused, reset, and unpaused successfully')
    else:
        print('Failed to pause, reset, or unpause simulation')


def set_controller_frequency(node: Node,desired_freq=5.0):
    # Create a client for the SetParameters service
    cli = node.create_client(SetParameters, "/controller_server/set_parameters")

    # Check service availability
    if not cli.wait_for_service(timeout_sec=10):
        print('Failed to connect to the set_parameters service')
        return

    # Create a request for the SetParameters service
    req = SetParameters.Request()

    # Create the parameter value
    param_value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=desired_freq)

    # Add the parameter to the request
    req.parameters.append(Parameter(name="controller_frequency", value=param_value))

    # Call the service
    future = cli.call_async(req)

    # Spin until the service call is complete
    rclpy.spin_until_future_complete(node,future)

    # Check the result of the service call
    if future.result() is not None:
        for result in future.result().results:
            if result.successful:
                print('Parameter set successfully')
            else:
                print(f'Failed to set parameter: {result.reason}')
    else:
        print('Service call failed')