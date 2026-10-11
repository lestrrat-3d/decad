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

## Shard manifests

- ALWAYS update `.github/test-shards.txt` after adding, renaming or removing a
  root-package test, fuzz target or example. Update
  `.github/test-shards-apitest.txt` for `apitest/` tests.
- ALWAYS run `go test . ./apitest/ -run '^TestCI'` before pushing any
  test-name change.
