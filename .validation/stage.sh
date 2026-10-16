#!/usr/bin/env bash
set -euo pipefail
variant=${1:?}; action=${2:?}; consumer=${3:-rods}; count=${4:-1}; mode=${5:-warm}; workers=${6:-1}
export GOMAXPROCS=2 GOFLAGS=-p=1 GOENV=off GOTOOLCHAIN=local
export GOCACHE=/out/go-cache GOMODCACHE=/out/go-mod-cache GOTMPDIR=/out/go-tmp GOPATH=/out/gopath
export VALIDATION_CONSUMER="$consumer" VALIDATION_MODE="$mode" VALIDATION_WORKERS="$workers"
mkdir -p "$GOCACHE" "$GOMODCACHE" "$GOTMPDIR"
test "$(cat /sys/fs/cgroup/memory.max)" = 4294967296
test "$(cat /sys/fs/cgroup/memory.swap.max)" = 0
test "$(cat /sys/fs/cgroup/pids.max)" = 64
test "$(cat /sys/fs/cgroup/cpu.max)" = '200000 100000'
cd "/out/$variant-src"
case "$action" in
 compile_api) exec go test -c -p=1 -o "/out/$variant-api.test" ./apitest ;;
 compile_helper) exec go test -c -p=1 -o "/out/$variant-helper.test" ./internal/meshbool ;;
 admission) cd apitest; exec "/out/$variant-api.test" -test.run '^TestValidationConsumerAdmission$' -test.v -test.parallel=1 -test.count=1 -test.timeout=45s ;;
 census) cd apitest; exec "/out/$variant-api.test" -test.run '^TestValidationOwnerCensus$' -test.v -test.parallel=1 -test.count=1 -test.timeout=45s ;;
 consumer) cd apitest; exec "/out/$variant-api.test" -test.run '^$' -test.bench '^BenchmarkValidationOwnerConsumer$' -test.benchtime="${count}x" -test.benchmem -test.count=1 -test.timeout=45s ;;
 helper) cd internal/meshbool; exec "/out/$variant-helper.test" -test.run '^$' -test.bench '^BenchmarkValidationOwnerHelper$' -test.benchtime=100x -test.benchmem -test.count=1 -test.timeout=45s ;;
 heap_consumer) cd apitest; exec "/out/$variant-api.test" -test.run '^TestValidationConsumerHeap$' -test.v -test.parallel=1 -test.count=1 -test.timeout=45s ;;
 heap_helper) cd internal/meshbool; exec "/out/$variant-helper.test" -test.run '^TestValidationOwnerHelperHeap$' -test.v -test.parallel=1 -test.count=1 -test.timeout=45s ;;
 cancel) cd internal/meshbool; exec "/out/$variant-helper.test" -test.run '^TestValidationOwnerCancellationTiming$' -test.v -test.parallel=1 -test.count=1 -test.timeout=45s ;;
 *) exit 64 ;;
esac
