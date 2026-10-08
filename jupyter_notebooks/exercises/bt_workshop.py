"""Small observable leaves for the behaviour tree exercises.

No decisions about task priority, retry or cancellation belong in these leaves.
The notebooks compose those decisions with py_trees composites and decorators.
"""

from dataclasses import dataclass, field
import operator
import uuid

import py_trees

Status = py_trees.common.Status


@dataclass
class World:
    battery_low: bool = False
    scan_requested: bool = False
    cancel_requested: bool = False
    active: bool = False
    result: str | None = None
    sensors_enabled: bool = False
    led: str | None = None
    events: list = field(default_factory=list)
    namespace: str = field(default_factory=lambda: '/workshop_' + uuid.uuid4().hex)


class Condition(py_trees.behaviour.Behaviour):
    def __init__(self, name, predicate):
        super().__init__(name)
        self.predicate = predicate

    def update(self):
        return Status.SUCCESS if self.predicate() else Status.FAILURE


class Record(py_trees.behaviour.Behaviour):
    def __init__(self, name, world):
        super().__init__(name)
        self.world = world

    def update(self):
        self.world.events.append((self.name, 'success'))
        return Status.SUCCESS


class SimAction(py_trees.behaviour.Behaviour):
    """A finite action; each entry sends one goal, each update polls once."""
    def __init__(self, name, world, ticks=3, outcomes=None):
        super().__init__(name)
        self.world = world
        self.ticks = ticks
        self.outcomes = iter(outcomes or [Status.SUCCESS])
        self.remaining = 0

    def initialise(self):
        self.remaining = self.ticks
        self.outcome = next(self.outcomes, Status.SUCCESS)
        self.world.events.append((self.name, 'start'))

    def update(self):
        self.remaining -= 1
        self.world.events.append((self.name, 'poll'))
        return Status.RUNNING if self.remaining > 0 else self.outcome

    def terminate(self, new_status):
        if self.status == Status.RUNNING and new_status == Status.INVALID:
            self.world.events.append((self.name, 'cancel'))
        elif new_status in (Status.SUCCESS, Status.FAILURE):
            self.world.events.append((self.name, new_status.name.lower()))


class Led(py_trees.behaviour.Behaviour):
    def __init__(self, name, world, colour):
        super().__init__(name)
        self.world = world
        self.colour = colour

    def update(self):
        self.world.led = self.colour
        self.world.events.append((self.name, 'running'))
        return Status.RUNNING

    def terminate(self, new_status):
        if self.world.led == self.colour:
            self.world.led = None
        self.world.events.append((self.name, 'clear'))


class SensorScope(py_trees.decorators.Decorator):
    """Local simulated context, restored on success, failure and interruption.

    This flag is deliberately local. A real ROS parameter needs asynchronous
    service calls, as demonstrated by ScanContext in the vendored tutorial.
    """
    def __init__(self, world, child):
        super().__init__('Sensors while scanning', child)
        self.world = world

    def initialise(self):
        self.previous = self.world.sensors_enabled
        self.world.sensors_enabled = True
        self.world.events.append(('sensors', 'enable'))

    def update(self):
        return self.decorated.status

    def terminate(self, new_status):
        self.world.sensors_enabled = self.previous
        self.world.events.append(('sensors', 'restore'))


class InputsToBlackboard(py_trees.behaviour.Behaviour):
    """Latch one-shot scan/cancel inputs, then publish a snapshot for this tick.

    Ignore a cancel in idle, ignore additional scans during a task, and discard
    a simultaneous scan/cancel in idle. A low battery retains the active request
    until the emergency ends. This is an explicit input contract, not a queue.
    """
    def __init__(self, world):
        super().__init__('Inputs -> blackboard')
        self.world = world
        self.bb = self.attach_blackboard_client(namespace=world.namespace)
        for key in ['battery_low', 'active', 'cancel_requested']:
            self.bb.register_key(key, access=py_trees.common.Access.WRITE)

    def update(self):
        w = self.world
        if not w.active:
            if w.scan_requested and not w.cancel_requested:
                w.active = True
                w.result = None
            w.cancel_requested = False
        w.scan_requested = False
        self.bb.battery_low = w.battery_low
        self.bb.active = w.active
        self.bb.cancel_requested = w.cancel_requested
        return Status.SUCCESS


def bb_is(world, key):
    return py_trees.behaviours.CheckBlackboardVariableValue(
        name=key + '?',
        check=py_trees.common.ComparisonExpression(
            variable=world.namespace + '/' + key, value=True, operator=operator.eq
        ),
    )


class Finish(py_trees.behaviour.Behaviour):
    def __init__(self, world, result):
        super().__init__('Report ' + result)
        self.world = world
        self.result = result

    def update(self):
        self.world.result = self.result
        self.world.active = False
        self.world.cancel_requested = False
        self.world.events.append(('result', self.result))
        # A report succeeded even when it reports an unsuccessful task.
        return Status.SUCCESS


def tick(root, count=1):
    for _ in range(count):
        root.tick_once()
    return root.status


def count(world, name, event):
    return world.events.count((name, event))


def check_reactivity(builder):
    w = World()
    root = builder(w)
    try:
        tick(root)
        assert root.tip().name == 'Patrol', 'Patrol powinien działać przy dobrej baterii'
        w.battery_low = True
        tick(root)
        assert root.tip().name == 'Alarm', 'Alarm musi przerwać patrol w następnym ticku'
        assert count(w, 'Patrol', 'cancel') == 1, 'Przerwanie musi anulować akcję'
        w.battery_low = False
        tick(root)
        assert root.tip().name == 'Patrol', 'Warunek alarmu musi być sprawdzany ponownie'
        assert count(w, 'Patrol', 'start') == 2, 'Patrol powinien wznowić pracę'
    finally:
        root.stop(Status.INVALID)
    print('OK: patrol -> alarm -> patrol; anulowanie dokładnie raz')


def check_parallel(builder):
    w = World()
    root = builder(w)
    try:
        assert tick(root) == Status.RUNNING
        assert count(w, 'Warning', 'success') == 1, 'Ostrzeżenie musi się wykonać'
        assert w.led == 'red'
        assert tick(root) == Status.RUNNING
        assert count(w, 'Warning', 'success') == 1, 'Ostrzeżenie tylko raz na wejście'
        assert tick(root) == Status.SUCCESS, 'LED RUNNING nie może blokować końca zadania'
        assert w.led is None, 'Zakończenie musi posprzątać LED'
    finally:
        root.stop(Status.INVALID)
    print('OK: ostrzeżenie raz, LED równolegle, koniec po sukcesie akcji')


def check_event_memory(builder):
    w = World(scan_requested=True)
    root = builder(w)
    try:
        assert tick(root) == Status.RUNNING
        w.scan_requested = False
        assert tick(root) == Status.RUNNING, 'Zdarzenie znika, lecz rozpoczęte zadanie trwa'
        assert tick(root) == Status.SUCCESS
        assert count(w, 'Scan', 'start') == 1
        assert tick(root) == Status.FAILURE, 'Bez nowego zdarzenia nie uruchamiaj skanu'
    finally:
        root.stop(Status.INVALID)
    print('OK: pojedyncze zdarzenie uruchamia zadanie do końca, bez powtórzenia')


def check_recovery(builder):
    for first, second, expected in [
        (Status.SUCCESS, Status.SUCCESS, Status.SUCCESS),
        (Status.FAILURE, Status.SUCCESS, Status.SUCCESS),
        (Status.FAILURE, Status.FAILURE, Status.FAILURE),
    ]:
        w = World()
        root = builder(SimAction('Scan', w, outcomes=[first]), Record('Repair', w),
                       SimAction('Retry', w, outcomes=[second]))
        try:
            for _ in range(8):
                if tick(root) != Status.RUNNING:
                    break
            assert root.status == expected, 'Po naprawie trzeba poczekać na wynik Retry'
            retries = int(first == Status.FAILURE)
            assert count(w, 'Scan', 'start') == 1, 'Pierwszej próby nie uruchamiaj ponownie podczas Retry'
            assert count(w, 'Repair', 'success') == retries, 'Naprawa tylko po błędzie i tylko raz'
            assert count(w, 'Retry', 'start') == retries, 'Dokładnie jedna ponowna próba po błędzie'
        finally:
            root.stop(Status.INVALID)
    print('OK recovery: sukces bez naprawy; naprawa + sukces; naprawa + drugi błąd')


def check_scanning(builder):
    for previous in [False, True]:
        for exit_kind in [Status.SUCCESS, Status.FAILURE, Status.INVALID]:
            w = World(sensors_enabled=previous)
            root = builder(w, SimAction('Scan', w, outcomes=[exit_kind]), Led('Blue', w, 'blue'))
            try:
                assert tick(root) == Status.RUNNING
                assert w.sensors_enabled and w.led == 'blue', 'Kontekst i LED muszą działać podczas skanu'
                if exit_kind == Status.INVALID:
                    root.stop(Status.INVALID)
                    assert count(w, 'Scan', 'cancel') == 1
                else:
                    tick(root)
                    assert tick(root) == exit_kind, 'Wynik skanu, a nie LED, kończy Parallel'
                assert w.sensors_enabled == previous, 'Przywróć poprzednią wartość kontekstu'
                assert w.led is None, 'Każde wyjście sprząta LED'
            finally:
                root.stop(Status.INVALID)
    print('OK zasoby: LED i kontekst aktywne podczas skanu, przywrócone po każdym wyjściu')


def check_mission(builder):
    """Exercise success, cancellation, emergency, retry and exhausted recovery."""
    cases = 0
    for outcomes, expected, starts in [
        ([Status.SUCCESS], 'success', 1),
        ([Status.FAILURE, Status.SUCCESS], 'success', 2),
        ([Status.FAILURE, Status.FAILURE], 'failed', 2),
    ]:
        w = World(scan_requested=True)
        scan = SimAction('Scan', w, outcomes=outcomes)
        retry = SimAction('Retry', w, outcomes=outcomes[1:])
        root = builder(w, scan, Led('Blue', w, 'blue'), Record('Repair', w), retry)
        try:
            tick(root, 9)
            assert w.result == expected, (w.events, w.result, expected)
            assert count(w, 'Scan', 'start') + count(w, 'Retry', 'start') == starts
            assert count(w, 'Repair', 'success') == starts - 1
            assert not w.active and not w.sensors_enabled and w.led is None
            assert count(w, 'result', expected) == 1, 'Raport tylko raz na żądanie'
        finally:
            root.stop(Status.INVALID)
        cases += 1
    for reason in ['cancel', 'battery']:
        w = World(scan_requested=True)
        root = builder(w, SimAction('Scan', w, ticks=6), Led('Blue', w, 'blue'),
                       Record('Repair', w), SimAction('Retry', w))
        try:
            tick(root)
            assert w.active and w.sensors_enabled and w.led == 'blue'
            if reason == 'cancel':
                w.cancel_requested = True
            else:
                w.battery_low = True
            tick(root)
            assert count(w, 'Scan', 'cancel') == 1, 'Priorytet musi anulować bieżącą akcję'
            assert not w.sensors_enabled and w.led != 'blue'
            if reason == 'cancel':
                assert w.result == 'cancelled' and not w.active
                tick(root, 3)
                assert count(w, 'result', 'cancelled') == 1
            else:
                assert w.active and w.result is None
                w.battery_low = False
                tick(root, 6)
                assert w.result == 'success'
                assert count(w, 'Scan', 'start') == 2
        finally:
            root.stop(Status.INVALID)
        cases += 1
    # A cancel received with no active task must not cancel the next request.
    w = World(cancel_requested=True)
    root = builder(w, SimAction('Scan', w), Led('Blue', w, 'blue'),
                   Record('Repair', w), SimAction('Retry', w))
    try:
        tick(root)
        assert w.result is None and not w.cancel_requested
        w.scan_requested = True
        tick(root, 3)
        assert w.result == 'success'
    finally:
        root.stop(Status.INVALID)
    print(f'OK: {cases + 1} scenariuszy: sukces, retry, błąd, cancel, bateria, cancel w idle')
