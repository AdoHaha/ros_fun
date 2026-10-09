"""Observed-motion checks for the beginner notebook verifier (not lesson code)."""
import math

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


def angle_difference(after, before):
    return math.atan2(math.sin(after - before), math.cos(after - before))


class PoseProbe:
    def __init__(self, lab):
        self.lab = lab
        self.pose = None
        self.samples = []
        self.subscription = lab.node.create_subscription(Pose, '/turtle1/pose', self.receive, 10)
        lab.wait_for(lambda: self.pose is not None, description='motion-check pose')

    def receive(self, pose):
        self.pose = pose
        self.samples.append(pose)

    def current(self):
        self.lab.spin_for(0.1)
        return self.pose

    def close(self):
        self.lab.node.destroy_subscription(self.subscription)


def begin_square(lab):
    probe = PoseProbe(lab)
    return probe, probe.current()


def check_square(probe, start):
    try:
        end = probe.current()
        closure = math.hypot(end.x - start.x, end.y - start.y)
        turn = abs(angle_difference(end.theta, start.theta))
        # Express all observations in the turtle's original body orientation.
        c, s = math.cos(start.theta), math.sin(start.theta)
        points = [(c * (p.x - start.x) + s * (p.y - start.y),
                   -s * (p.x - start.x) + c * (p.y - start.y)) for p in probe.samples]
        corners = ((1, 0), (1, 1), (0, 1), (0, 0))
        errors = [min(math.hypot(x - cx, y - cy) for x, y in points) for cx, cy in corners]
        diagnostic = {'closure': closure, 'heading_error': turn, 'corner_errors': errors,
                      'final_velocity': [end.linear_velocity, end.angular_velocity],
                      'start': [start.x, start.y, start.theta],
                      'end': [end.x, end.y, end.theta], 'samples': len(points)}
        print('SQUARE_OBSERVED', diagnostic)
        hint = 'Close Gazebo/RViz/rqt; turtlesim integrates fixed steps and can lag under load.'
        assert closure < 0.25, f'Square did not close: {closure:.3f}. {hint}'
        assert turn < 0.25, f'Square final heading error: {turn:.3f}. {hint}'
        assert max(errors) < 0.25, f'Expected square corners not reached: {errors}. {hint}'
        print('SQUARE_PASS', {'closure': closure, 'heading_error': turn, 'corner_errors': errors})
    finally:
        probe.close()


def check_panel(lab, buttons):
    probe = PoseProbe(lab)
    commands = []
    subscription = lab.node.create_subscription(Twist, '/turtle1/cmd_vel', commands.append, 10)
    try:
        for button, _ in buttons:
            before = probe.current()
            first_command = len(commands)
            button.click()  # invoke the actual registered widget callback
            after = probe.current()
            sent = commands[first_command:]
            assert sent, f'No ROS commands from {button.description}'
            assert sent[-1].linear.x == 0 and sent[-1].angular.z == 0, button.description
            assert abs(after.linear_velocity) < 0.01 and abs(after.angular_velocity) < 0.01
            dx, dy = after.x - before.x, after.y - before.y
            forward = dx * math.cos(before.theta) + dy * math.sin(before.theta)
            rotation = angle_difference(after.theta, before.theta)
            label = button.description
            if 'przód' in label:
                assert forward > 0.1, (label, forward)
            elif 'tył' in label:
                assert forward < -0.1, (label, forward)
            elif '←' in label:
                assert rotation > 0.2, (label, rotation)
            elif '→' in label:
                assert rotation < -0.2, (label, rotation)
            elif 'STOP' in label:
                assert math.hypot(dx, dy) < 0.03 and abs(rotation) < 0.03
            else:
                raise AssertionError(f'Unchecked button: {label}')
            print('BUTTON_PASS', label, {'forward': forward, 'rotation': rotation})
    finally:
        lab.node.destroy_subscription(subscription)
        probe.close()


def drive_checkpoints(lab, publisher, targets, visited, refresh):
    """Reach each checkpoint through normal velocity commands, never teleport."""
    probe = PoseProbe(lab)
    try:
        visited.clear()
        for name, (x, y) in targets.items():
            for _ in range(4):
                pose = probe.current()
                distance = math.hypot(x - pose.x, y - pose.y)
                if distance < 0.25:
                    break
                heading = math.atan2(y - pose.y, x - pose.x)
                delta = angle_difference(heading, pose.theta)
                lab.drive(publisher, angular=delta / 0.5, seconds=0.5)
                lab.drive(publisher, linear=1.5, seconds=min(distance / 1.5, 4.0))
            pose = probe.current()
            distance = math.hypot(x - pose.x, y - pose.y)
            assert distance < 0.6, (name, distance)
            assert name in visited, (name, visited)
            print('CHECKPOINT_DRIVE_PASS', name, {'distance': distance, 'score': len(visited)})
        assert visited == set(targets)
        lab.spin_for(0.2)
        assert len(visited) == len(targets), 'Stationary samples added duplicate points'
        refresh()
    finally:
        probe.close()
