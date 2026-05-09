import os

from glob import glob
from setuptools import find_packages, setup

package_name = 'ros_fun'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    package_data={
        'ros_fun_py_trees_ros_tutorials': [
            'LICENSE.upstream',
            'README.ros_fun.md',
            'mock/gui/*',
        ],
    },
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Mock robot components
            'mock-battery = ros_fun_py_trees_ros_tutorials.mock.battery:main',
            'mock-dashboard = ros_fun_py_trees_ros_tutorials.mock.dashboard:main',
            'mock-docking-controller = ros_fun_py_trees_ros_tutorials.mock.dock:main',
            'mock-led-strip = ros_fun_py_trees_ros_tutorials.mock.led_strip:main',
            'mock-move-base = ros_fun_py_trees_ros_tutorials.mock.move_base:main',
            'mock-rotation-controller = ros_fun_py_trees_ros_tutorials.mock.rotate:main',
            'mock-safety-sensors = ros_fun_py_trees_ros_tutorials.mock.safety_sensors:main',
            'mock-dock-client = ros_fun_py_trees_ros_tutorials.mock.actions:dock_client',
            'mock-move-base-client = ros_fun_py_trees_ros_tutorials.mock.actions:move_base_client',
            'mock-rotate-client = ros_fun_py_trees_ros_tutorials.mock.actions:rotate_client',
            # Tutorial trees
            'tree-data-gathering = ros_fun_py_trees_ros_tutorials.one_data_gathering:tutorial_main',
            'tree-battery-check = ros_fun_py_trees_ros_tutorials.two_battery_check:tutorial_main',
            'tree-action-clients = ros_fun_py_trees_ros_tutorials.five_action_clients:tutorial_main',
            'tree-context-switching = ros_fun_py_trees_ros_tutorials.six_context_switching:tutorial_main',
            'tree-docking-cancelling-failing = ros_fun_py_trees_ros_tutorials.seven_docking_cancelling_failing:tutorial_main',
            'tree-dynamic-application-loading = ros_fun_py_trees_ros_tutorials.eight_dynamic_application_loading:tutorial_main',
        ],
    },
)
