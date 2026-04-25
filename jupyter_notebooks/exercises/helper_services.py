from std_srvs.srv import Empty
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import ParameterValue, ParameterType, Parameter
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
import rclpy
import subprocess
import time


WORKSHOP_PROCESS_PATTERNS = [
    "ros2 launch turtlebot3_gazebo",
    "ros2 launch turtlebot3_navigation2",
    "ros2 launch nav2_bringup",
    "component_container_isolated",
    "rviz2",
    "gz sim",
    "parameter_bridge",
    "image_bridge",
    "robot_state_publisher",
]


def cleanup_workshop_processes(wait_sec=2.0):
    """Stop stale workshop ROS/Gazebo processes before relaunching simulation."""
    for pattern in WORKSHOP_PROCESS_PATTERNS:
        subprocess.run(["pkill", "-f", pattern], check=False)

    subprocess.run(["ros2", "daemon", "stop"], check=False)
    time.sleep(wait_sec)


def turtlebot3_nav2_command(
    map_path="/home/ubuntu/turtlebot3_ws/src/jupyter_notebooks/map.yaml",
):
    """Return the Jazzy Nav2 command with TurtleBot3 params and stamped cmd_vel."""
    return (
        "ros2 launch nav2_bringup bringup_launch.py autostart:=True use_sim_time:=True "
        f"map:={map_path} "
        "params_file:=$(ros2 pkg prefix turtlebot3_navigation2)/share/"
        "turtlebot3_navigation2/param/waffle_pi.yaml"
    )


def turtlebot3_rviz_command():
    return (
        "ros2 run rviz2 rviz2 -d $(ros2 pkg prefix turtlebot3_navigation2)/share/"
        "turtlebot3_navigation2/rviz/tb3_navigation2.rviz --ros-args -p use_sim_time:=true"
    )


def publish_initial_pose(
    node: Node,
    x=0.08,
    y=0.0,
    orientation_z=0.0,
    orientation_w=1.0,
    repeats=10,
):
    """Publish AMCL initial pose with QoS compatible with Jazzy Nav2."""
    initial_pose_qos = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
    )
    publisher = node.create_publisher(
        PoseWithCovarianceStamped, "initialpose", initial_pose_qos
    )
    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.stamp = rclpy.time.Time().to_msg()
    initial_pose.header.frame_id = "map"
    initial_pose.pose.pose.position.x = x
    initial_pose.pose.pose.position.y = y
    initial_pose.pose.pose.orientation.z = orientation_z
    initial_pose.pose.pose.orientation.w = orientation_w
    initial_pose.pose.covariance[0] = 0.25
    initial_pose.pose.covariance[7] = 0.25
    initial_pose.pose.covariance[-1] = 0.06

    for _ in range(repeats):
        publisher.publish(initial_pose)
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.1)


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
