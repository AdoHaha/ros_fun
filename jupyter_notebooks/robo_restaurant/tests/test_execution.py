"""Headless execution contract checks using the actual restaurant planner."""
from dataclasses import replace
import unittest

from robo_restaurant.execution import ActionExecutor, Outcome, Status
from robo_restaurant.planning import Action, State, execute, goal, solve


class ExecutionTests(unittest.TestCase):
    def finish(self, executor, limit=20):
        for _ in range(limit):
            status = executor.tick()
            if status is not Status.RUNNING:
                return status
        self.fail('scripted action did not finish within its tick limit')

    def test_delay_does_not_apply_effects_until_success(self):
        initial = State()
        executor = ActionExecutor(initial)
        action = Action('move', 'kitchen')
        self.assertIs(executor.start(action, Outcome(running_ticks=3)), Status.RUNNING)
        self.assertEqual(executor.state, initial)
        for _ in range(3):
            self.assertIs(executor.tick(), Status.RUNNING)
            self.assertEqual(executor.state, initial)
        self.assertIs(executor.tick(), Status.SUCCESS)
        self.assertEqual(executor.state, execute(initial, action))

    def test_terminal_ticks_do_not_apply_effects_twice(self):
        executor = ActionExecutor()
        executor.start(Action('move', 'kitchen'), Outcome(running_ticks=0))
        self.assertIs(executor.tick(), Status.SUCCESS)
        final = executor.state
        for _ in range(5):
            self.assertIs(executor.tick(), Status.SUCCESS)
            self.assertEqual(executor.state, final)
        self.assertEqual(len(executor.dispatches), 1)

    def test_failure_preserves_state_and_local_retry_repeats_action(self):
        initial = State()
        executor = ActionExecutor(initial)
        action = Action('move', 'kitchen')
        executor.start(action, Outcome(success=False, running_ticks=1))
        self.assertIs(self.finish(executor), Status.FAILURE)
        self.assertEqual(executor.state, initial)
        self.assertTrue(executor.can_retry)
        executor.retry(Outcome(running_ticks=1))
        self.assertIs(self.finish(executor), Status.SUCCESS)
        self.assertEqual(executor.dispatches, [action, action])
        self.assertEqual(executor.state.battery, 6)

    def test_local_retry_is_bounded(self):
        executor = ActionExecutor()
        executor.start(Action('move', 'kitchen'), Outcome(success=False, running_ticks=0))
        executor.tick()
        executor.retry(Outcome(success=False, running_ticks=0))
        executor.tick()
        self.assertFalse(executor.can_retry)
        with self.assertRaises(RuntimeError):
            executor.retry()
        self.assertEqual(len(executor.dispatches), 2)
        self.assertEqual(executor.state, State())

    def test_illegal_dispatch_is_rejected_without_changing_state(self):
        executor = ActionExecutor()
        with self.assertRaises(ValueError):
            executor.start(Action('serve', '1'))
        self.assertEqual(executor.state, State())
        self.assertIs(executor.status, Status.IDLE)
        self.assertEqual(executor.dispatches, [])

    def test_running_action_cannot_be_overwritten(self):
        executor = ActionExecutor()
        executor.start(Action('move', 'kitchen'))
        with self.assertRaises(RuntimeError):
            executor.start(Action('move', 'table_1'))
        self.assertEqual(executor.action, Action('move', 'kitchen'))

    def test_cooker_failure_requires_repair_and_replan_from_observed_state(self):
        initial = State(location='kitchen', battery=6)
        observed = replace(initial, cooker_ok=False)
        executor = ActionExecutor(initial)
        executor.start(Action('prepare', '1'), Outcome(
            success=False, running_ticks=2, observed=observed, message='Cooker fault'))
        self.assertIs(self.finish(executor), Status.FAILURE)
        self.assertEqual(executor.state, observed)
        self.assertEqual(executor.state.meals, ('ordered', 'ordered'))
        self.assertFalse(executor.can_retry)
        with self.assertRaises(RuntimeError):
            executor.retry()
        plan, cost = solve(executor.state)
        self.assertEqual(plan[0], Action('repair'))
        self.assertEqual(cost, 28)
        for action in plan:
            executor.start(action, Outcome(running_ticks=1))
            self.assertIs(self.finish(executor), Status.SUCCESS)
        self.assertTrue(goal(executor.state))

    def test_observation_invalidates_pending_success(self):
        initial = State(location='kitchen', battery=6)
        executor = ActionExecutor(initial)
        executor.start(Action('prepare', '1'), Outcome(running_ticks=3))
        self.assertIs(executor.tick(), Status.RUNNING)
        observed = replace(initial, cooker_ok=False)
        executor.observe(observed)
        self.assertIs(executor.tick(), Status.FAILURE)
        self.assertEqual(executor.state, observed)
        self.assertFalse(executor.can_retry)
        self.assertEqual(executor.failure_reason, 'state_changed')

    def test_unchanged_observation_does_not_interrupt_action(self):
        executor = ActionExecutor()
        executor.start(Action('move', 'kitchen'), Outcome(running_ticks=0))
        executor.observe(State())
        self.assertIs(executor.tick(), Status.SUCCESS)

    def test_changed_resources_require_replan_even_if_action_remains_legal(self):
        executor = ActionExecutor()
        observed = State(battery=4)
        executor.start(Action('move', 'kitchen'), Outcome(
            success=False, running_ticks=0, observed=observed))
        self.assertIs(executor.tick(), Status.FAILURE)
        execute(observed, executor.action)  # still legal, but old plan assumptions changed
        self.assertFalse(executor.can_retry)
        self.assertEqual(executor.failure_reason, 'state_changed')

    def test_observation_after_failure_also_invalidates_local_retry(self):
        executor = ActionExecutor()
        executor.start(Action('move', 'kitchen'), Outcome(success=False, running_ticks=0))
        executor.tick()
        self.assertTrue(executor.can_retry)
        executor.observe(State(battery=4))
        self.assertFalse(executor.can_retry)
        self.assertEqual(executor.failure_reason, 'state_changed')

    def test_observation_does_not_resume_a_cancelled_mission(self):
        executor = ActionExecutor()
        executor.start(Action('move', 'kitchen'))
        executor.cancel()
        executor.observe(State(battery=4))
        self.assertEqual(executor.failure_reason, 'cancelled')
        self.assertFalse(executor.can_retry)

    def test_cancel_discards_pending_effects_and_does_not_retry(self):
        executor = ActionExecutor()
        executor.start(Action('move', 'kitchen'))
        executor.cancel()
        self.assertIs(executor.tick(), Status.FAILURE)
        self.assertFalse(executor.can_retry)
        self.assertEqual(executor.state, State())

    def test_unreachable_observed_state_is_not_reported_as_success(self):
        executor = ActionExecutor()
        executor.start(Action('move', 'table_1'), Outcome(
            success=False, observed=State(location='table_1', battery=0)))
        self.assertIs(self.finish(executor), Status.FAILURE)
        self.assertFalse(executor.can_retry)
        self.assertEqual(solve(executor.state), (None, None))
        self.assertFalse(goal(executor.state))

    def test_all_reference_scenarios_complete_after_acknowledgements(self):
        for initial in (State(), State(battery=2), State(cooker_ok=False)):
            with self.subTest(initial=initial):
                executor = ActionExecutor(initial)
                plan, _ = solve(initial)
                for action in plan:
                    executor.start(action, Outcome(running_ticks=2))
                    self.assertIs(self.finish(executor), Status.SUCCESS)
                self.assertTrue(goal(executor.state))

    def test_malformed_scripted_outcomes_are_rejected(self):
        for arguments in ({'running_ticks': -1}, {'running_ticks': 0.5},
                          {'success': 'yes'}, {'success': True, 'observed': State()}):
            with self.subTest(arguments=arguments), self.assertRaises(ValueError):
                Outcome(**arguments)


if __name__ == '__main__':
    unittest.main()
