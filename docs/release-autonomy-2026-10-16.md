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

Independent source review passed: 66 tests, six BT notebooks, planning starter
demonstrations 12–15, actual turtlesim action completion/cancellation and parameter
CLI acceptance/rejection. Candidate-image checks and the fresh student run are
pending. The previous day's student pass is historical evidence, not the final
merged-repository acceptance run.
