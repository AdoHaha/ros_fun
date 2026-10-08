"""Reference answers to the three notebook programming exercises."""
from ..planning import execute, goal, solve, successors


def audit_plan(initial, plan):
    """Replay a plan, reject illegal steps, and return final state and cost."""
    state, cost = initial, 0
    for action in plan:
        transition = next((item for item in successors(state) if item[0] == action), None)
        if transition is None:
            raise ValueError(f'Illegal action: {action}')
        _, state, step_cost = transition
        cost += step_cost
    return state, cost


def energy_report(initial):
    """Return feasibility, optimal cost and number of charging actions."""
    plan, cost = solve(initial)
    return {'feasible': plan is not None, 'cost': cost,
            'charges': None if plan is None else sum(a.name == 'charge' for a in plan)}


def recover(observed):
    """Replace a stale plan with one solved from the latest observation."""
    plan, cost = solve(observed)
    if plan is None:
        return {'status': 'blocked', 'plan': None, 'cost': None}
    final, actual_cost = audit_plan(observed, plan)
    assert goal(final) and actual_cost == cost
    return {'status': 'ready', 'plan': plan, 'cost': cost}


def run_world_solution(scenario, world):
    """Execute a solution and verify Gazebo after each action and external event."""
    from dataclasses import replace
    from ..planning import State
    if scenario not in ('normal', 'low_battery', 'cooker_failure'):
        raise ValueError(scenario)
    state = State(battery=2 if scenario == 'low_battery' else 8)
    world.sync(state)
    result = recover(state)
    plan = list(result['plan'])
    injected, cost, steps = False, 0, 0
    while not goal(state):
        if not plan:
            raise RuntimeError('No plan; manager intervention required')
        action = plan.pop(0)
        if scenario == 'cooker_failure' and action.name == 'prepare' and not injected:
            state = replace(state, cooker_ok=False)
            world.sync(state)
            injected = True
            result = recover(state)
            if result['status'] != 'ready':
                raise RuntimeError('No recovery plan')
            plan = list(result['plan'])
            continue
        following, step_cost = audit_plan(state, [action])
        world.sync(following)
        state = following
        cost += step_cost
        steps += 1
    return {'state': state, 'cost': cost, 'steps': steps, 'failure_injected': injected}
