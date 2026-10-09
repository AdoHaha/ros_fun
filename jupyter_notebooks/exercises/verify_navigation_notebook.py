#!/usr/bin/env python3
"""Execute exercise 6 against owned Gazebo/Nav2, checking actual arrival and stop.

Run inside the sourced workshop container. Uses ROS domain 95; do not use that
domain simultaneously for a student session. Outputs stay outside the checkout.
"""
import argparse
import json
import os
from pathlib import Path

import nbformat
from nbclient import NotebookClient


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--solutions', action='store_true')
    parser.add_argument('--repeat', type=int, default=1)
    parser.add_argument('--action-bridge', action='store_true',
                        help='Also execute exercise 7\'s optional native Nav2 ActionClient cell')
    parser.add_argument('--output', type=Path, default=Path('/tmp/navigation-notebook-verification'))
    args = parser.parse_args()
    if args.repeat < 1:
        parser.error('--repeat must be positive')
    os.environ['ROS_DOMAIN_ID'] = '95'
    root = Path(__file__).resolve().parent
    args.output.mkdir(parents=True, exist_ok=True)
    notebook = nbformat.read(root / '6. Navigate - nav2.ipynb', as_version=4)
    answers = json.loads((root / '.solutions/navigation_answers.json').read_text())
    for cell in notebook.cells:
        if args.solutions and cell.metadata.get('course_task') in answers:
            cell.source = answers[cell.metadata.course_task]
        tags = cell.metadata.get('tags', [])
        if 'course6-arrival' in tags:
            cell.source = 'arrival_pose_samples_before = navigation.pose_samples\n' + cell.source
            cell.source += '''
assert wynik == TaskResult.SUCCEEDED
navigation.wait_for(lambda: navigation.pose_samples > arrival_pose_samples_before,
                    description='lokalizacja odebrana podczas przejazdu')
navigation.spin_for(0.5)
navigation.wait_for(lambda: navigation.map_position() is not None, description='bieżąca pozycja map z TF')
x, y = navigation.map_position()
arrival_error = math.hypot(x-ODBIOR[0], y-ODBIOR[1])
assert arrival_error < 0.35, arrival_error
assert navigation.path_length > 0.4 and navigation.nonzero_commands > 5
print('ARRIVAL_OBSERVED_PASS', arrival_error, navigation.path_length)
'''
        if 'course6-cancel' in tags:
            cell.source += '''
assert wynik_anulowania == TaskResult.CANCELED
cancel_odom_before = navigation.odom_samples
navigation.wait_for(lambda: navigation.odom_samples > cancel_odom_before
                    and abs(navigation.odom.twist.twist.linear.x) < 0.02
                    and abs(navigation.odom.twist.twist.angular.z) < 0.02,
                    timeout=20, description='robot zatrzymany po anulowaniu')
assert not navigation.active
print('CANCEL_TERMINAL_STOP_PASS')
'''
        if 'course6-route' in tags:
            cell.source += ('\nassert wyniki_trasy == [TaskResult.SUCCEEDED]*2\n'
                            "print('DELIVERY_ROUTE_PASS')") if args.solutions else '\nassert wyniki_trasy == []'
    if args.action_bridge:
        action_notebook = nbformat.read(root / '7. ROS Action.ipynb', as_version=4)
        action_source = next(cell.source for cell in action_notebook.cells
                             if cell.cell_type == 'code' and 'RUN_NAV2 = False' in cell.source)
        # Use the exact optional client cell with the already verified map goal.
        action_source = action_source.replace('RUN_NAV2 = False', 'RUN_NAV2 = True')
        action_source = action_source.replace('NAV_X, NAV_Y, NAV_YAW = 0.0, 0.0, 0.0',
                                              'NAV_X, NAV_Y, NAV_YAW = 0.5, -1.2, -math.pi/2')
        bridge = nbformat.v4.new_code_cell('''
from intro_ros import IntroLab
from action_workshop import close_action_lab, wait_action_future
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
bridge_lab = IntroLab('courier_nav2_action_bridge')
bridge_namespace = dict(lab=bridge_lab, node=bridge_lab.node, proby_akcji=[],
                        nav_action_client=None, ActionClient=ActionClient,
                        wait_action_future=wait_action_future, close_action_lab=close_action_lab,
                        math=math, STATUSY={GoalStatus.STATUS_SUCCEEDED: 'SUKCES'})
try:
    exec(ACTION_BRIDGE_SOURCE, bridge_namespace)
    assert bridge_namespace['handle_nav'].accepted
    assert bridge_namespace['wynik_nav'].status == GoalStatus.STATUS_SUCCEEDED
    navigation.spin_for(0.5)
    navigation.wait_for(lambda: navigation.map_position() is not None)
    x, y = navigation.map_position()
    bridge_error = math.hypot(x-ODBIOR[0], y-ODBIOR[1])
    assert bridge_error < 0.35, bridge_error
    print('NATIVE_NAV2_ACTION_BRIDGE_PASS', bridge_error)
finally:
    close_action_lab(bridge_namespace)
'''.replace('ACTION_BRIDGE_SOURCE', repr(action_source)))
        cleanup_index = next(index for index, cell in enumerate(notebook.cells)
                             if 'course6-cleanup' in cell.metadata.get('tags', []))
        notebook.cells.insert(cleanup_index, bridge)
    client = NotebookClient(notebook, timeout=660, kernel_name='python3',
                            resources={'metadata': {'path': str(root)}})
    dest = args.output / ('solved.ipynb' if args.solutions else 'student.ipynb')
    with client.setup_kernel(cwd=str(root)):
        try:
            for run in range(args.repeat):
                for index, cell in enumerate(notebook.cells):
                    print(f'RUN {run+1} CELL {index} {cell.cell_type}', flush=True)
                    client.execute_cell(cell, index, execution_count=client.code_cells_executed+1)
        finally:
            try:
                cleanup = nbformat.v4.new_code_cell("if 'navigation' in globals():\n    navigation.close()")
                notebook.cells.append(cleanup)
                client.execute_cell(cleanup, len(notebook.cells)-1)
            finally:
                nbformat.write(notebook, dest)
    report = {'result': 'passed', 'solutions': args.solutions, 'runs_per_kernel': args.repeat,
              'native_action_bridge': args.action_bridge}
    dest.with_suffix('.json').write_text(json.dumps(report, indent=2)+'\n')
    print('NAVIGATION_NOTEBOOK_PASS', report, flush=True)


if __name__ == '__main__':
    main()
