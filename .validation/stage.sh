#!/usr/bin/env bash
set -euo pipefail
variant=${1:?}; action=${2:?}; case_name=${3:-rods}; count=${4:-1}
export GOMAXPROCS=2 GOFLAGS=-p=1 GOENV=off GOTOOLCHAIN=local
export GOCACHE=/out/go-cache GOMODCACHE=/out/go-mod-cache GOTMPDIR=/out/go-tmp GOPATH=/out/gopath
export VALIDATION_CONSUMER="$case_name" VALIDATION_CASE="$case_name"
mkdir -p "$GOCACHE" "$GOMODCACHE" "$GOTMPDIR"
test "$(cat /sys/fs/cgroup/memory.max)" = 4294967296
test "$(cat /sys/fs/cgroup/memory.swap.max)" = 0
test "$(cat /sys/fs/cgroup/pids.max)" = 64
test "$(cat /sys/fs/cgroup/cpu.max)" = '200000 100000'
cd "/out/$variant-src"
case "$action" in
 compile_api) exec go test -c -p=1 -o "/out/$variant-api.test" ./apitest ;;
 compile_helper) exec go test -c -p=1 -o "/out/$variant-helper.test" ./internal/meshbool ;;
 compile_race) exec go test -c -race -p=1 -o "/out/$variant-race.test" ./internal/meshbool ;;
 admission) cd apitest; exec "/out/$variant-api.test" -test.run '^TestValidationConsumerAdmission$' -test.v -test.parallel=1 -test.count=1 -test.timeout=45s ;;
 consumer) cd apitest; exec "/out/$variant-api.test" -test.run '^$' -test.bench '^BenchmarkValidationIncidenceConsumer$' -test.benchtime="${count}x" -test.benchmem -test.count=1 -test.timeout=45s ;;
 constructor) cd internal/meshbool; exec "/out/$variant-helper.test" -test.run '^$' -test.bench '^BenchmarkValidationIncidenceConstructor$' -test.benchtime=100x -test.benchmem -test.count=1 -test.timeout=30s ;;
 cancel) cd internal/meshbool; exec "/out/$variant-helper.test" -test.run '^TestValidationIncidenceCancellationLatency$' -test.v -test.parallel=1 -test.count=1 -test.timeout=30s ;;
 correctness|race)
  pattern='^(TestEnforceHeldEmbeddingMovesAFlattenedSliver|TestEnforceHeldEmbeddingRefusesWhenNoCornerClears|TestEnforceHeldEmbeddingRefusesTheUnion23Fold)$'
  if test "$variant" = candidate; then pattern='^(TestHeldIncidenceMatchesAppend|TestHeldIncidenceInvalidIndex|TestHeldIncidenceSize|TestHeldIncidenceGenerationAndStamps|TestHeldIncidenceBudgetAndCancellation|TestHeldIncidenceCancellationPollOrder|TestEnforceHeldEmbeddingMovesAFlattenedSliver|TestEnforceHeldEmbeddingRefusesWhenNoCornerClears|TestEnforceHeldEmbeddingRefusesTheUnion23Fold)$'; fi
  binary=helper
  if test "$action" = race; then binary=race; fi
  cd internal/meshbool
  exec "/out/$variant-$binary.test" -test.run "$pattern" -test.v -test.parallel=1 -test.count=1 -test.timeout=30s ;;
 *) exit 64 ;;
esac
