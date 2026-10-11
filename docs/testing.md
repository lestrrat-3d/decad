# Test guidance

Read this file before adding, renaming or removing tests, fuzz targets or
examples, or changing CI shards.

## Test scope

- Keep Decad's geometry and failure behavior covered by small, focused tests.
  Split a test that dominates a CI shard into tests for its separate operations.
- Put full consumer models, including complete gears, in the consumer repository.
  Test the underlying Decad operations here with minimal geometry.
- Preserve computed-geometry assertions when replacing a large test. Do not
  drop coverage just to shorten CI.

## Shard manifests

- ALWAYS update `.github/test-shards.txt` or `.github/test-shards-apitest.txt`
  after adding, renaming or removing a test, fuzz target or example.
- ALWAYS run `go test . ./apitest/ -run '^TestCI'` before pushing any
  test-name change.
