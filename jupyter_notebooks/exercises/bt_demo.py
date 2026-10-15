#!/usr/bin/env python3
"""Run a deterministic BT demo before starting ROS or Gazebo.

python3 bt_demo.py
python3 bt_demo.py --memory   # compare an unresponsive priority selector
"""
import argparse

import py_trees
from bt_workshop import World, Condition, Led, SimAction, Status


def make_demo(world, memory=False):
    alarm = py_trees.composites.Sequence('Battery guard', memory=False)
    alarm.add_children([Condition('Battery low?', lambda: world.battery_low), Led('Alarm', world, 'red')])
    priorities = py_trees.composites.Selector('Reactive priorities', memory=memory)
    priorities.add_children([alarm, SimAction('Patrol step', world, ticks=4)])
    return priorities


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--memory', action='store_true')
    args = parser.parse_args()
    world = World()
    root = make_demo(world, args.memory)
    try:
        for i, low in enumerate([False, False, True, True, False, False, False, False]):
            world.battery_low = low
            root.tick_once()
            print(f'\nTick {i}: battery_low={low}, root={root.status.name}, LED={world.led}')
            print(py_trees.display.unicode_tree(root, show_status=True))
    finally:
        root.stop(Status.INVALID)
    print('Events:', world.events)


if __name__ == '__main__':
    main()
