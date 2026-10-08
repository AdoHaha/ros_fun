#!/usr/bin/env python3
"""Execute the complete Nav2 notebook and verify motion, goal, patrol and stop.

Run inside the workshop container with ROS sourced. Launches the existing castle
simulation through the notebook cells; allow several minutes on a slow host.
"""
import argparse
import json
from pathlib import Path
import nbformat
from nbclient import NotebookClient

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output', type=Path, default=Path('/tmp/bt-nav2-verification'))
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=True)
base = Path(__file__).resolve().parent
n=nbformat.read(base/'11. Behavior Trees with Nav2 Helper Demo.ipynb',as_version=4)
validation=nbformat.v4.new_code_cell('''
import json
from rclpy.executors import SingleThreadedExecutor
monitor = trees_nav.MotionMonitor()
monitor_executor = SingleThreadedExecutor()
monitor_executor.add_node(monitor)
monitor_running = True
def spin_monitor():
    while monitor_running:
        monitor_executor.spin_once(timeout_sec=0.05)
monitor_thread = threading.Thread(target=spin_monitor, daemon=True)
monitor_thread.start()
def wait_demo(predicate, timeout):
    deadline=time.monotonic()+timeout
    last_report=0
    while time.monotonic()<deadline:
        if time.monotonic()-last_report>10:
            print('MONITOR',monitor.summary(),status_text(),flush=True)
            verification_output.joinpath('progress.json').write_text(json.dumps({'motion':monitor.summary(),'events':state.event_log()},indent=2))
            last_report=time.monotonic()
        if runner.last_error:
            raise runner.last_error
        if predicate():
            return
        time.sleep(0.1)
    raise TimeoutError(str(state.event_log()))
try:
    wait_demo(lambda: monitor.nonzero_cmd_samples >= 20, 40)
    go_to_goal()
    wait_demo(lambda: any(e['event']=='goal_cleared_to_patrol' for e in state.event_log()), 360)
    cleared=next(e['time'] for e in state.event_log() if e['event']=='goal_cleared_to_patrol')
    wait_demo(lambda: any(e['event']=='patrol_goal_sent' and e['time']>cleared for e in state.event_log()), 10)
    stop_robot()
    wait_demo(lambda: demo.navigation_owner is None and any(e['event']=='cancel_initialise' for e in state.event_log()), 10)
    time.sleep(1.0)
    summary=monitor.summary()
    assert summary['last_cmd'] == (0.0,0.0), summary
    assert summary['path_length_m'] >= 0.5, summary
    assert runner.last_error is None
    print('LIVE_NAV2_PASS', json.dumps(summary))
    verification_output.joinpath('result.json').write_text(json.dumps({'motion':summary,'events':state.event_log()},indent=2))
finally:
    monitor_running=False
    monitor_thread.join(timeout=2)
    monitor_executor.remove_node(monitor)
    monitor_executor.shutdown()
    monitor.destroy_node()
''')
validation.source = 'verification_output = Path(' + repr(str(args.output.resolve())) + ')\n' + validation.source
n.cells.insert(-1,validation)
client=NotebookClient(n,timeout=430,kernel_name='python3',resources={'metadata':{'path':str(base)}})
client.reset_execution_trackers()
with client.setup_kernel(cwd=str(base)):
    try:
        for index,cell in enumerate(n.cells):
            print(f'CELL {index} {cell.cell_type}',flush=True)
            if index == len(n.cells) - 1:
                continue  # cleanup also runs after failures while the kernel is alive
            client.execute_cell(cell,index,execution_count=client.code_cells_executed+1)
    finally:
        try:
            client.execute_cell(n.cells[-1],len(n.cells)-1,execution_count=client.code_cells_executed+1)
        finally:
            nbformat.write(n,args.output / 'executed.ipynb')
print('NAV2 NOTEBOOK PASSED',flush=True)
