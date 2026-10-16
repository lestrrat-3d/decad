#!/usr/bin/env bash
set -euo pipefail
variant=${1:?}; action=${2:?}; case_name=${3:-none}
export GOMAXPROCS=2 GOFLAGS=-p=1 GOENV=off GOTOOLCHAIN=local
export GOCACHE=/out/go-cache GOMODCACHE=/out/go-mod-cache GOTMPDIR=/out/go-tmp GOPATH=/out/gopath
export VALIDATION_CASE="$case_name"
mkdir -p "$GOCACHE" "$GOMODCACHE" "$GOTMPDIR"
test "$(cat /sys/fs/cgroup/memory.max)" = 4294967296
test "$(cat /sys/fs/cgroup/memory.swap.max)" = 0
test "$(cat /sys/fs/cgroup/pids.max)" = 64
test "$(cat /sys/fs/cgroup/cpu.max)" = '200000 100000'
cd "/out/$variant-src"
case "$action" in
 compile_helper) exec go test -c -p=1 -o "/out/$variant-helper.test" ./internal/meshbool ;;
 compile_race) exec go test -c -race -p=1 -o "/out/$variant-race.test" ./internal/meshbool ;;
 cancel) cd internal/meshbool; exec "/out/$variant-helper.test" -test.run '^TestValidationConformCancellationLatency$' -test.v -test.parallel=1 -test.count=1 -test.timeout=30s ;;
 heap) cd internal/meshbool; exec "/out/$variant-helper.test" -test.run '^TestValidationConformRetainedHeap$' -test.v -test.parallel=1 -test.count=1 -test.timeout=30s ;;
 correctness|race)
  pattern='^(TestStitchMapTetrahedronFixture|TestStitchFacetsClosureRefusals)$'
  if test "$variant" = candidate; then pattern='^(TestConformUnchangedOutputMatchesOriginal|TestConformUnchangedOutputCancellationMatchesOriginal|TestStitchMapTetrahedronFixture|TestStitchFacetsClosureRefusals)$'; fi
  binary=helper
  if test "$action" = race; then binary=race; fi
  cd internal/meshbool
  exec "/out/$variant-$binary.test" -test.run "$pattern" -test.v -test.parallel=1 -test.count=1 -test.timeout=30s ;;
 *) exit 64 ;;
esac
