# Performance improvement workflow

Use this workflow to find and improve one measurable Decad performance problem.

## Check CI runs

- Inspect recent `main` and pull request runs for `.github/workflows/ci.yml` with `gh`.
- Compare workflow duration, job duration, and per-test duration separately.
- Read shard logs for `runner-cpu` and `--- PASS` timing lines.
- Compare per-test timings only when runner CPU labels match.
- Treat queue and artifact-build time separately from job execution time.
- Repeat a surprising slowdown before treating it as a regression.
- Select one reproducible slowdown or code path for investigation.

## Measure one workload

- Find an existing focused Go benchmark for the selected operation.
- If none exists, add the smallest benchmark that exercises the real operation and representative input.
- Record the exact commit, benchmark command, Go version, machine, and result.
- Compare baseline and candidate with the same machine, input, flags, and Go version.
- Run enough repetitions to distinguish a change from timing noise.
- Use CPU or allocation profiles only when benchmark results do not locate the cost.
- Do not use a full-suite runtime as evidence for a local code hotspot.
- Stop if the slowdown cannot be reproduced or the candidate does not improve the measured workload.

## Design and implement

- Give a `gpt-6-sol` subagent the measured evidence, relevant source and design documents, and the intended scope.
- Ask it for a root-cause explanation, a narrow design, correctness risks, and a focused verification plan.
- Review the design against `CLAUDE.md` and the owning `agent-docs/*-design.md` before implementation.
- Give the reviewed design and benchmark evidence to a `gpt-6-luna` subagent for implementation.
- Keep one performance issue in each implementation and pull request.
- Preserve public behavior and computed-geometry assertions while changing the implementation.

## Verify and merge

- Run focused tests and the same benchmark before opening a pull request.
- Compare baseline and candidate results from the recorded workload and machine.
- Follow `agent-docs/testing.md` for test scope and shard-manifest changes.
- Follow repository Git instructions for commit and pull request message lint, Go lint, and CI watching.
- Open one pull request with the slowdown evidence, benchmark command, before-and-after results, and correctness checks.
- Merge with `gh pr merge <number> --squash` only after required checks and reviews are ready and the measured workload improves.
- Do not merge when checks fail, review is pending, or the measured result is neutral or worse.

## Keep the next check useful

- Record the CI run links, runner CPU labels, selected slowdown, benchmark command and environment, before-and-after results, pull request, and outcome in the pull request.
- On the next invocation, inspect newer runs and prior performance pull requests before repeating an investigation.
- This document does not schedule itself. Run it periodically when a user or configured scheduler invokes the workflow.
