# Test guidance

Read this file before adding, renaming or removing tests, fuzz targets or
examples, or changing CI shards.

## Test scope

- Keep Decad's geometry and failure behavior covered by small, focused tests.
  Split a test that dominates a CI shard into tests for its separate operations.
- Put full consumer models, including complete gears, in the consumer repository.
  Test the underlying Decad operations here with minimal geometry.
- If CI renders a full consumer model, use that render as its integration check.
  Keep exact geometry and proof assertions in focused Decad tests instead of
  adding another full-model test.
- Preserve computed-geometry assertions when replacing a large test. Do not
  drop coverage just to shorten CI.
- CI runs every test under `-race`. A differential sweep may run a smaller case
  set there through the package's `raceDetector` constant
  (`race_enabled_test.go` / `race_disabled_test.go`); the plain build keeps the
  full sweep. Before shrinking a set, remove each guard the test protects and
  confirm the smaller set still fails.

## Shard manifests

- ALWAYS update `.github/test-shards.txt` after adding, renaming or removing a
  root-package test, fuzz target or example. Update
  `.github/test-shards-apitest.txt` for `apitest/` tests.
- ALWAYS run `go test . ./apitest/ -run '^TestCI'` before pushing any
  test-name change.
- A new test that has not been measured goes in shard 10, never in the lightest
  shard. Shards are rebalanced only by regenerating them with `_shardgen` from
  measured costs.

## PR execution

- Run example packages whole. `examples/thread/basic/` owns thread forms,
  handedness and lookup. `examples/thread/leadin/` owns thread lead-in geometry.
  Other examples stay in `examples/`. The independent examples job runs the
  whole subtree with Go's package parallelism, without test-name assignments.
- Keep independent internal-package jobs and dynamics independent of the
  race-binary artifact. The workflow's dedicated-package exclusions must match
  its dedicated runners; `TestCIWorkflowRaceShardsCoverEveryPackage` checks them.
- Measure PR completion from workflow creation through its last required check.
  Job execution budgets exclude runner queue and artifact-build time. A passing
  budget does not establish the five-minute PR target.

## Per-test timing in CI

Each test binary's `TestMain` (`ci_verbose_test.go`, `apitest/ci_verbose_test.go`)
turns on `-test.v` when `GITHUB_ACTIONS` is `true`, so shard logs carry one
`--- PASS: Name (x.xxs)` line per test. Keep it the only `TestMain` in its package.
