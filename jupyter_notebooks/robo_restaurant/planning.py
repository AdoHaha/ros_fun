"""Deterministic restaurant domain and uniform-cost reference planner.

Run from jupyter_notebooks: python3 -m robo_restaurant.planning --scenario normal
Movement is symbolic; it does not command a Gazebo robot.
"""
import argparse
from dataclasses import dataclass, replace
from heapq import heappop, heappush
from itertools import count

LOCATIONS = ('dock', 'kitchen', 'table_1', 'table_2')
# Abstract travel energy/cost, not geometric distances. Every edge is reversible.
EDGES = {('dock', 'kitchen'): 2, ('dock', 'table_1'): 2,
         ('table_1', 'table_2'): 2, ('kitchen', 'table_2'): 2}


@dataclass(frozen=True)
class State:
    location: str = 'dock'
    battery: int = 8
    carried: int = -1
    meals: tuple = ('ordered', 'ordered')
    cooker_ok: bool = True


@dataclass(frozen=True)
class Action:
    name: str
    target: str = ''


def successors(state):
    """Yield legal (action, next_state, cost) transitions.

    A single cook prepares meals synchronously; the tray holds one meal.
    Battery is bounded so the search space remains finite.
    """
    for (a, b), energy in EDGES.items():
        destination = b if state.location == a else a if state.location == b else None
        if destination is not None and state.battery >= energy:
            yield Action('move', destination), replace(
                state, location=destination, battery=state.battery-energy), energy
    if state.location == 'dock' and state.battery < 8:
        yield Action('charge'), replace(state, battery=8), 3
    if state.location == 'kitchen':
        if not state.cooker_ok and state.carried == -1:
            yield Action('repair'), replace(state, cooker_ok=True), 5
        for index, meal in enumerate(state.meals):
            meals = list(state.meals)
            if meal == 'ordered' and state.cooker_ok:
                meals[index] = 'ready'
                yield Action('prepare', str(index+1)), replace(state, meals=tuple(meals)), 3
            if meal == 'ready' and state.carried == -1:
                meals[index] = 'carried'
                yield Action('pick_up', str(index+1)), replace(
                    state, meals=tuple(meals), carried=index), 1
    if state.carried >= 0 and state.location == f'table_{state.carried+1}':
        meals = list(state.meals)
        meals[state.carried] = 'served'
        yield Action('serve', str(state.carried+1)), replace(
            state, meals=tuple(meals), carried=-1), 1


def goal(state):
    return state.meals == ('served', 'served') and state.location == 'dock'


def solve(initial):
    """Dijkstra search: minimum total action cost, with explicit no-plan result."""
    serial = count()
    frontier = [(0, next(serial), initial)]
    costs, parents = {initial: 0}, {}
    while frontier:
        cost, _, state = heappop(frontier)
        if cost != costs[state]:
            continue
        if goal(state):
            plan = []
            while state != initial:
                previous, action = parents[state]
                plan.append(action)
                state = previous
            return list(reversed(plan)), cost
        for action, following, step_cost in successors(state):
            candidate = cost + step_cost
            if candidate < costs.get(following, float('inf')):
                costs[following] = candidate
                parents[following] = state, action
                heappush(frontier, (candidate, next(serial), following))
    return None, None


def execute(state, action):
    for legal, following, _ in successors(state):
        if action == legal:
            return following
    raise ValueError(f'Action {action} is no longer applicable in {state}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', choices=('normal', 'low_battery', 'cooker_failure'), default='normal')
    args = parser.parse_args()
    state = State(battery=2 if args.scenario == 'low_battery' else 8)
    plan, cost = solve(state)
    print(f'Initial state: {state}\nInitial plan cost: {cost}')
    injected = False
    while not goal(state):
        if not plan:
            raise RuntimeError('No feasible plan; manager intervention required')
        action = plan.pop(0)
        # Inject a device event before the first preparation, after travel.
        if args.scenario == 'cooker_failure' and action.name == 'prepare' and not injected:
            state = replace(state, cooker_ok=False)
            injected = True
            print('EVENT: cooker failed; discard remaining plan and replan')
            plan, cost = solve(state)
            print(f'Recovery plan cost: {cost}')
            continue
        state = execute(state, action)
        print(f'{action.name:8} {action.target:8} battery={state.battery} meals={state.meals}')
    print('SUCCESS: both customers served and waiter returned to dock')


if __name__ == '__main__':
    main()
