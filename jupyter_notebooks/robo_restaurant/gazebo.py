"""Verified discrete visualization of symbolic states in Gazebo.

This adapter moves visual proxies via UserCommands. It does not simulate driving,
collisions, cooking time, or sensors. The planner remains the state authority.
"""
from contextlib import contextmanager
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import tempfile
import time
import uuid

WORLD = Path(__file__).parent / 'worlds' / 'restaurant.sdf'
SERVICE_XY = {'dock': (0., -3.5), 'kitchen': (-2.4, 2.),
              'table_1': (1.6, -2.), 'table_2': (1.6, 2.)}


def command(args, env=None, timeout=20):
    return subprocess.run(['gz', *args], env=env, capture_output=True, text=True,
                          check=True, timeout=timeout).stdout


def blocks(text, field, indent=0):
    """Extract known protobuf text message blocks at a specified indentation."""
    for match in re.finditer(r'^' + ' '*indent + re.escape(field) + r' \{', text, re.M):
        start, depth = match.end(), 1
        end = start
        while depth and end < len(text):
            depth += (text[end] == '{') - (text[end] == '}')
            end += 1
        if depth:
            raise ValueError('Incomplete Gazebo response')
        yield text[start:end-1]


def scene_positions(text):
    result = {}
    for model in blocks(text, 'model'):
        name = re.search(r'^  name: "([^"]+)"', model, re.M)
        pose = next(blocks(model, 'pose', 2), '')
        position = next(blocks(pose, 'position', 4), '')
        xyz = []
        for axis in 'xyz':
            value = re.search(r'^      ' + axis + r': ([^\n]+)', position, re.M)
            xyz.append(float(value.group(1)) if value else 0.)
        if name:
            result[name.group(1)] = tuple(xyz)
    if not result:
        raise ValueError('Scene response contains no model positions')
    return result


def expected_positions(state):
    x, y = SERVICE_XY[state.location]
    poses = {'waiter_proxy': (x, y, .35),
             'cooker_fault': (-4., 3.6, 1.3 if not state.cooker_ok else -2.)}
    for index, meal in enumerate(state.meals):
        if meal == 'ordered':
            pose = (-4. + .5*index, 2., -2.)
        elif meal == 'ready':
            pose = (-4. + .5*index, 2., 1.1)
        elif meal == 'carried':
            pose = (x, y, .8)
        elif meal == 'served':
            pose = (3., -2. if index == 0 else 2., 1.2)
        else:
            raise ValueError(f'Unknown meal state: {meal}')
        poses[f'meal_{index+1}'] = pose
    return poses


def live_positions(text):
    result = {}
    for pose in blocks(text, 'pose'):
        name = re.search(r'^  name: "([^"]+)"', pose, re.M)
        position = next(blocks(pose, 'position', 2), '')
        xyz = []
        for axis in 'xyz':
            value = re.search(r'^    ' + axis + r': ([^\n]+)', position, re.M)
            xyz.append(float(value.group(1)) if value else 0.)
        if name:
            result[name.group(1)] = tuple(xyz)
    return result


class GazeboRestaurant:
    def __init__(self, env=None):
        self.env = env

    def scene(self):
        text = command(['service', '-s', '/world/robo_restaurant/scene/info',
                        '--reqtype', 'gz.msgs.Empty', '--reptype', 'gz.msgs.Scene',
                        '--timeout', '5000', '--req', ''], self.env)
        return scene_positions(text)

    def sync(self, state):
        """Apply a state to the visual scene and verify Gazebo's returned poses."""
        expected = expected_positions(state)
        request = ' '.join(
            f'pose {{ name: "{name}" position {{ x: {x} y: {y} z: {z} }} '
            'orientation { w: 1 } }'
            for name, (x, y, z) in expected.items())
        reply = command(['service', '-s', '/world/robo_restaurant/set_pose_vector/blocking',
                         '--reqtype', 'gz.msgs.Pose_V', '--reptype', 'gz.msgs.Boolean',
                         '--timeout', '5000', '--req', request], self.env)
        if 'data: true' not in reply:
            raise RuntimeError(f'Gazebo rejected state update: {reply}')
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            # scene/info describes the initial scene; pose/info carries updates.
            actual = live_positions(command(
                ['topic', '-e', '-t', '/world/robo_restaurant/pose/info', '-n', '1'],
                self.env, timeout=10))
            if all(name in actual and all(math.isclose(a, b, abs_tol=1e-6)
                   for a, b in zip(actual[name], xyz)) for name, xyz in expected.items()):
                return actual
            time.sleep(.05)
        raise AssertionError(f'Scene does not reflect state: expected {expected}, observed {actual}')


@contextmanager
def isolated_restaurant():
    """Start a private headless server and always stop it on exit."""
    env = dict(os.environ, GZ_PARTITION=f'restaurant-{uuid.uuid4().hex}')
    with tempfile.TemporaryFile(mode='w+') as log:
        server = subprocess.Popen(['gz', 'sim', '-s', '-r', str(WORLD)], env=env,
                                  stdout=log, stderr=log, start_new_session=True)
        try:
            deadline = time.monotonic() + 30
            while time.monotonic() < deadline:
                if server.poll() is not None:
                    raise RuntimeError('Gazebo exited during startup')
                services = command(['service', '-l'], env)
                if '/world/robo_restaurant/set_pose/blocking' in services and '/world/robo_restaurant/scene/info' in services:
                    break
                time.sleep(.1)
            else:
                raise TimeoutError('Restaurant services did not start')
            yield GazeboRestaurant(env)
        finally:
            if server.poll() is None:
                os.killpg(server.pid, signal.SIGINT)
                try:
                    server.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(server.pid, signal.SIGKILL)
                    server.wait(timeout=5)
            log.seek(0)
            messages = log.read()
            if '[Err]' in messages:
                raise RuntimeError(messages)
