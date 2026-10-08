# Robo-restaurant: planning lab starter

You are the restaurant manager. A waiter serves robot customers with energy
cartridges (the meals). The cook, charging dock and cooker constrain which
orders can be completed. Students select actions, execute them, observe events
and replace plans when their assumptions become false.

## Run the starter

From `jupyter_notebooks` (the notebook server's root):

```bash
gz sim -r robo_restaurant/worlds/restaurant.sdf
python3 -m robo_restaurant.planning --scenario normal
python3 -m robo_restaurant.planning --scenario low_battery
python3 -m robo_restaurant.planning --scenario cooker_failure
```

The world targets modern Gazebo (Harmonic in the Jazzy workshop), contains
primitive geometry only and needs no Fuel downloads. There is a 12 × 10 m room,
kitchen, cook robot, two tables with robot customers, washing station and dock.
The cook and customers are static visual props in this first version. There is
a visual waiter proxy, two meal markers, and a red cooker-fault marker. The
terminal demos run independently of Gazebo. Instructor notebooks can update
the running world and verify each state against live Gazebo poses. The waiter
moves discretely between service poses; there is no navigation stack or ROS
state publisher yet. This milestone tests task effects before adding driving.

## Student notebooks and separate instructor solutions

Open the `exercises/` folder in Jupyter and continue with notebooks 12–14:

1. [Normal service](<../exercises/12. Robo Restaurant - Normal Service.ipynb>): audit a plan and reject illegal actions.
2. [Low battery](<../exercises/13. Robo Restaurant - Low Battery.ipynb>): report charging requirements and infeasibility.
3. [Cooker failure](<../exercises/14. Robo Restaurant - Cooker Failure.ipynb>): recover from an observed failure.

Each notebook contains a runnable demonstration, a TODO function and checks.
Unfinished answers print TODO; they are not counted as passed exercises.
Instructor answers are maintained outside the student checkout.

## Repeatable student validation

Inside the sourced Jazzy workshop container, from `jupyter_notebooks`:

```bash
python3 -m unittest discover -s robo_restaurant/tests -v
python3 robo_restaurant/tools/verify_notebooks.py --output /tmp/restaurant-notebooks
```

The runner executes the three student demonstrations and confirms that unanswered
exercise cells remain marked TODO. Executed copies go to the output folder.
The separate instructor checkout contains completed solutions and full Gazebo
integration checks. See [review](REVIEW.md) for the validation performed before
student distribution.

Service poses in world coordinates (metres; yaw chosen by the navigation adapter):

| Symbol | x | y |
| --- | ---: | ---: |
| dock | 0 | -3.5 |
| kitchen | -2.4 | 2 |
| table_1 | 1.6 | -2 |
| table_2 | 1.6 | 2 |

The abstract graph's edges specify legal travel and illustrative energy costs;
they are not collision-free trajectories or measured distances. Keep route
planning in Nav2 and task planning in the manager.

## Planning model

The immutable state records waiter location, battery (0–8), carried meal,
order states and cooker availability. Each meal progresses through `ordered`,
`ready`, `carried`, `served`. The tray holds one meal. Actions are `move`,
`prepare`, `pick_up`, `serve`, `charge`, `repair`; `successors` defines their
preconditions, effects and costs. Cooking and repair are synchronous abstract
actions. Only travel consumes battery in this introductory model. Goal: serve
both customers and return to the dock. The reference solver uses uniform-cost
search to minimize the sum of travel and handling costs, not elapsed time.

The failure demo injects a cooker failure immediately before the first
preparation. It discards the remaining plan, solves from the observed state,
and repairs the cooker. The low-battery demo requires charging. This makes
resource constraints and replanning visible with reproducible traces.

## Suggested class (two 90-minute sessions)

1. **Model and inspect (20 min):** explore Gazebo, identify resources, write
   preconditions/effects on paper. Predict why a fixed action sequence fails.
2. **Normal service (25 min):** run the reference search; explain the plan and
   compare its cost with a hand-written valid plan. Explain why returning to
   the dock is part of the goal.
3. **Energy constraint (25 min):** start with battery 2; explain charging.
   Change the capacity and travel costs consistently. Find an infeasible
   configuration and report `no plan` rather than claiming success.
4. **Failure recovery (20 min):** run the injected failure. Explain which
   precondition broke and why replanning must start from the observed state.
5. **Extend the domain (45 min):** add `dirty` tables and a `clean` action at
   the washing station. Customers may only place another order after cleaning.
   Change the goal and demonstrate a plan that respects the new constraint.
6. **Planning backend (30 min):** adapt the same state and successor function
   to scikit-decide, or express the deterministic problem in PDDL using
   Unified Planning. Compare returned plans and objective values.
7. **Discuss and assess (15 min):** submit model, successful traces, one
   impossible case, and an explanation of recovery. Assess legal execution,
   goal satisfaction, resource accounting and clear failure reporting.

Advanced exercises: asynchronous cook states (`idle`, `busy`, `ready`,
`failed`), deadlines with explicit simulation time, recipe compatibility,
blocked routes, probabilistic repair, or two waiters sharing a cooker. Add one
feature at a time. Deadlines require a temporal model: action costs alone do
not represent concurrency or time.

## Planner choice and integration roadmap

Scikit-decide is a good next backend when the course should progress from
search to stochastic decision-making. Its documented solvers include A*,
MDP algorithms and MCTS; it also integrates Unified Planning/PDDL:
https://airbus.github.io/scikit-decide/
For a course focused on symbolic preconditions and effects, PDDL through
Unified Planning is also a suitable entry point. The starter deliberately has
no planner dependency; backend adapters are a subsequent milestone.

For a live ROS/Gazebo lab, add these components:

- Spawn the workshop's TurtleBot waiter at the dock and configure Nav2 against
  this world. Resolve service symbols through the coordinate table above.
- Create a restaurant state node, driven by simulation time, emitting orders,
  cook/device states and action outcomes. Include a monotonically increasing
  state revision and a scenario seed for reproducibility.
- Dispatch `move` through `NavigateToPose`; implement preparation, loading,
  serving and repair as acknowledged restaurant actions. Apply effects only
  after successful completion. Treat failed navigation as an observed event.
- Replan on changed preconditions or action failure. Reject stale commands;
  never mark an order served merely because a plan contains `serve`.
- Retain a headless symbolic mode for grading. Record success rate, cost,
  missed deadlines (once modelled) and replans across fixed scenarios.

Keep the planner, state model and executor separate. Existing behaviour-tree
exercises in this repository can supply the execution layer: the planner
selects a task sequence; the tree handles action completion and recovery.

World structure follows the Gazebo Harmonic SDF tutorial:
https://gazebosim.org/docs/harmonic/sdf_worlds/
