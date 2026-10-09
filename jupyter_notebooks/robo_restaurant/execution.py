"""Acknowledged symbolic execution for the planning/behaviour-tree bridge.

No robots, ROS clients, threads or clocks are created. ``running_ticks`` models
waiting for an action result, not duration in seconds. A failed simulated action
has no partial physical effects; a supplied observation is authoritative state.
"""
from dataclasses import dataclass
from enum import Enum

from .planning import Action, State, execute


class Status(Enum):
    IDLE = 'IDLE'
    RUNNING = 'RUNNING'
    SUCCESS = 'SUCCESS'
    FAILURE = 'FAILURE'


@dataclass(frozen=True)
class Outcome:
    """Controlled result: wait N ticks, then acknowledge success or failure.

    ``observed`` is allowed on failure to report an external event, such as a
    broken cooker. It does not mean that the failed action's planned effects ran.
    """
    success: bool = True
    running_ticks: int = 2
    observed: State | None = None
    message: str = ''

    def __post_init__(self):
        if type(self.success) is not bool:
            raise ValueError('success must be a boolean')
        if type(self.running_ticks) is not int or self.running_ticks < 0:
            raise ValueError('running_ticks must be a nonnegative integer')
        if self.observed is not None and not isinstance(self.observed, State):
            raise ValueError('observed must be a restaurant State')
        if self.success and self.observed is not None:
            raise ValueError('successful scripted results use the legal planned effects')


class ActionExecutor:
    """Execute one action at a time, with one bounded local retry by default.

    ``start`` checks preconditions but does not apply effects. ``tick`` applies
    effects once, only when the controlled result acknowledges success. Repeated
    terminal ticks return the recorded result. Call ``start`` for the next action.
    """
    def __init__(self, state=State(), max_retries=1):
        if not isinstance(state, State):
            raise TypeError('state must be a restaurant State')
        if type(max_retries) is not int or max_retries < 0:
            raise ValueError('max_retries must be a nonnegative integer')
        self._state = state
        self.max_retries = max_retries
        self.status = Status.IDLE
        self.action = None
        self.message = ''
        self.failure_reason = None
        self.retries = 0
        self.dispatches = []
        self._outcome = None
        self._remaining = 0

    @property
    def state(self):
        return self._state

    def _dispatch(self, action, outcome):
        if not isinstance(action, Action) or not isinstance(outcome, Outcome):
            raise TypeError('dispatch requires an Action and Outcome')
        execute(self.state, action)  # validate; discard the hypothetical successor
        self.action = action
        self._outcome = outcome
        self._remaining = outcome.running_ticks
        self.status = Status.RUNNING
        self.message = 'Waiting for acknowledgement'
        self.failure_reason = None
        self.dispatches.append(action)

    def start(self, action, outcome=Outcome()):
        if self.status is Status.RUNNING:
            raise RuntimeError('an action is already running')
        self._dispatch(action, outcome)
        self.retries = 0
        return self.status

    def tick(self):
        if self.status is not Status.RUNNING:
            return self.status
        if self._remaining:
            self._remaining -= 1
            return Status.RUNNING
        outcome = self._outcome
        self._outcome = None
        if outcome.success:
            self._state = execute(self.state, self.action)
            self.status = Status.SUCCESS
            self.message = outcome.message or 'Action acknowledged; effects committed'
        else:
            # Observed events replace belief state; failed action effects do not.
            changed = outcome.observed is not None and outcome.observed != self.state
            if outcome.observed is not None:
                self._state = outcome.observed
            self.status = Status.FAILURE
            self.failure_reason = 'state_changed' if changed else 'action_failed'
            self.message = outcome.message or 'Action failed; planned effects not applied'
        return self.status

    @property
    def can_retry(self):
        if (self.status is not Status.FAILURE or self.failure_reason != 'action_failed'
                or self.retries >= self.max_retries):
            return False
        try:
            execute(self.state, self.action)
        except ValueError:
            return False
        return True

    def retry(self, outcome=Outcome()):
        """Dispatch the same still-legal action without generating a new plan."""
        if not self.can_retry:
            raise RuntimeError('local retry unavailable; inspect state and replan or stop')
        self._dispatch(self.action, outcome)
        self.retries += 1
        return self.status

    def observe(self, state):
        """Accept changed external state and invalidate a pending acknowledgement.

        The mock has no real actuator to cancel. In a ROS adapter, request cancel,
        await the terminal result, and reject stale results before dispatching again.
        """
        if not isinstance(state, State):
            raise TypeError('observation must be a restaurant State')
        if state == self.state:
            return
        self._state = state
        if self.status is Status.RUNNING:
            self._outcome = None
            self.status = Status.FAILURE
            self.failure_reason = 'state_changed'
            self.message = 'Observation changed during action; old result discarded'
        elif self.status is Status.FAILURE and self.failure_reason != 'cancelled':
            self.failure_reason = 'state_changed'
            self.message = 'Observation changed after failure; discard the old plan'

    def cancel(self):
        """Invalidate pending mock work; no planned effects are committed."""
        if self.status is Status.RUNNING:
            self._outcome = None
            self.status = Status.FAILURE
            self.failure_reason = 'cancelled'
            self.message = 'Mock action cancelled'
