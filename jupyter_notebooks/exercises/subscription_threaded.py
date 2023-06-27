"""
Subscription class for jupyter-ros2 Project

Author: zmk5 (Zahi Kakish)

"""
from typing import TypeVar
from typing import Callable
import threading
import functools
import ipywidgets as widgets

try:
    import rclpy
    from rclpy.node import Node
except ModuleNotFoundError:
    print("The rclpy package is not found in your $PYTHONPATH. " +
          "Subscribe and publish are not going to work.")
    print("Do you need to activate your ros2 environment?")


# Used for documentation purposes only
MsgType = TypeVar('MsgType')


class ThreadedSpinner():
  
    def __init__(self, node: Node) -> None:
    
        self.node = node

 


    def __thread_target(self) -> None:
        while self.__thread_state:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.__widgets["out"].append_stdout("Done!\n")

    def stop(self, _) -> None:
        self.__thread_state = False

    def spin_in_thread(self) -> None:
        self.__thread_state = True
        local_thread = threading.Thread(target=self.__thread_target)
        local_thread.start()

