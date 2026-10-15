# Final review and release — 2026-10-16

## Integration plan

1. Preserve the updated courier and autonomy lessons from `origin/master`
   (`4493399`), including the new execution bridge in exercise 15. Keep planning
   exercises 12–14 in English, with the state/action introduction from our review.
2. Resolve notebook conflicts semantically, restore the full contents list,
   review the combined sources and run their available checks.
3. Build an immutable candidate for amd64 and arm64. Check NumPy/OpenCV,
   `cv_bridge`, plotting, ROS and the installed Jupyter widget assets. Use
   `Dockerfile_upgrade` over the previous published multiarch digest to avoid
   reinstalling all ROS packages; `Dockerfile` remains the full build recipe.
4. Freeze the code and run a new student agent without conversation history,
   review reports or instructor answers on the final repository and candidate
   image. Check its solutions independently. If a new defect requires code
   changes, retest the affected path before promotion.
5. Merge to `master`, publish the validated candidate on Docker Hub as `jazzy`
   with an immutable release tag, and verify remote commit and image digests.

## Findings resolved before candidate build

- The remote master had newer, substantially rewritten exercises. Its newer
  in-cell tasks and bounded helpers are retained; the old tutorial workflow is
  not reintroduced.
- Planning exercises 12–14 remain English, including the new explanation of
  observed state and retry versus replanning. Links lead on to exercise 15.
- The introduction links directly to exercises 9–15. README language information
  matches the notebooks.
- Runtime `PYTHONPATH` selects Ubuntu's NumPy for OpenCV/cv_bridge. During
  nbextension installation it is unset so the pip frontend package supplies
  its actual JavaScript assets instead of Ubuntu's asset-less Python shim.

## Verification status

The integrated executable source is frozen at
`d61bf73db61bcb5e831b6413327e1e3a356c2a4c`, which has `4493399` from remote
master as its second parent. Subsequent release-report and README edits do not
change the exercises or image recipe. Integration PR:
<https://github.com/AdoHaha/ros_fun/pull/11>.

- Independent source review passed 66 tests, six BT notebook executions, planning
  starter demonstrations 12–15, actual turtlesim action completion/cancellation
  and parameter CLI acceptance/rejection.
- The native amd64 candidate passed **138 tests**: 117 exercise tests and 21
  restaurant domain/execution tests. Additional test files came from the test
  branch in a separate verification overlay; executable sources came from the
  final integrated repository.
- Candidate smoke checks passed for NumPy 1.26.4, OpenCV 4.6.0, a `cv_bridge`
  roundtrip, matplotlib PNG generation, ROS imports and installed widget assets.
  These checks passed on native amd64 and on arm64 under QEMU. The arm64 local
  image config digest matches the published arm64 manifest's config digest.
- All 131 tracked workshop files embedded in the native candidate match the
  frozen source commit by SHA-256.
- A real candidate Jupyter server served widget JavaScript over HTTP and its
  live kernel emitted widget MIME data and handled a widget click via comms.
  This was protocol-level verification. Visual browser verification was not
  completed because the available browser runtime failed to initialize.

Native candidate tested by the new student:
`adohaha/fun_ros@sha256:1d94c22904ed5d76616fd15e82fdc2b441df60208db3b355655860f0ff46438e`.

Validated arm64 manifest:
`adohaha/fun_ros@sha256:c79e5e050492a86e72da5e60a770c45b8aa38dccee44316843c400a1c02e661a`.

## Fresh student acceptance

A new agent, without conversation history, received an isolated snapshot of the
integrated source and the native candidate digest. Instructor solutions, review
reports and private test cases were withheld; the `make_fake_demo` teaching
helper imported by notebook 11 was retained. The agent wrote its own solutions
and extra measurements. No runtime helper source was changed in its snapshot.
All **60 original notebook assertions** remain present.

| Exercises | Evidence from the new run |
| --- | --- |
| 6 | Real Nav2 delivery and return both `SUCCEEDED`; cancellation `CANCELED`; 2.482 m measured travel, 1064 odometry samples, final velocity `(0, 0)`. |
| 7 | Four compass directions completed in turtlesim; action feedback, result and cancellation observed. CLI inspected while the server was running. Optional Nav2 appendix was skipped. |
| 8 | Parameter validation, atomic rejection, movement/radius rules and actual CLI accept/reject checks passed. |
| 9–10 | Student compositions and original checks passed, including ROS mock success/cancel/alarm-resume. A second reviewer ran six additional reference tests against the student's actual builders: 6/6 passed. |
| 11 A | Independent recovery composition and assertions for ownership, memory mutations, bounded retry, goal completion and patrol index/cycles passed. |
| 11 B | Final run: GOAL interrupted patrol, succeeded and resumed patrol; the next patrol point succeeded. STOP then interrupted the following active, moving goal. 2.754 m travel, 928 odometry samples, 1255 nonzero commands. Terminal cancellation was `CANCELED`; commands and odometry velocities reached zero. |
| 12–14 | Completed notebook checks passed in fresh candidate-container kernels. Independent grading passed 32 tests plus partial/empty plans, input preservation, 18 energy/cooker configurations, completed and stranded states. |
| 15 | Completed notebook checks passed in the candidate container. Additional grading covered 18 observed states, blocked/completed/retry/cancel cases, invalid statuses and full executor immutability. |

The first live-11 attempt failed in the student's added measurement code because
it spun a shared ROS executor concurrently. The student diagnosed and corrected
its own monitor without changing the exercise helpers; the failed attempt was
preserved. Review of the second run found that STOP occurred after a patrol goal
had already succeeded. This proves the drive/priority/resume sequence but does
not by itself prove interruption of an active goal. A third probe recorded STOP
during motion but timed out on the student's assumption that the internal
`cancellation_pending` flag clears without polling while stopped. The fourth
run explicitly collected the terminal result after stopping the tree runner:
`CANCELED`, zero velocities, and less than 0.01 m displacement during a two-second
settling observation (measured approximately 6.6e-14 m). No new goal followed
STOP. Independent inspection confirmed active ownership, nonzero odometry and
an unfinished action result before STOP, followed by cancellation and rest.
All ten final executed notebooks (6–15) have zero execution errors.

An extra host-only attempt found that exercise 15 cannot run on Python 3.8.
Acceptance uses the workshop container's Python 3.12; the starter README now
states the supported runtime and accurately describes the notebook languages.

The previous day's student pass remains historical evidence and is not used as
the final merged-repository acceptance run.

## Docker Hub release

The combined candidate index contains the exact tested native manifest and the
validated arm64 manifest (with its build attestation):

`sha256:bce3b93e6885a06e22df212025bb094c7b75e360559162fa64b1607a2a9fec46`.

Candidate tag: `adohaha/fun_ros:jazzy-rc-20261016-d61bf73`. The accepted release
is promoted unchanged to stable `jazzy` and immutable `jazzy-20261016-d61bf73`.
The final merge and registry tags must be checked against these recorded source
and image digests after promotion.
Arm64 smoke tests ran under QEMU; the full student simulations ran on amd64.
