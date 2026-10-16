#!/usr/bin/env bash
set -euo pipefail
variant=${1:?}; action=${2:?}; case_name=${3:?}; count=${4:-1}
export GOMAXPROCS=2 GOFLAGS=-p=1 GOENV=off GOTOOLCHAIN=local
export GOCACHE=/out/go-cache GOMODCACHE=/out/go-mod-cache GOTMPDIR=/out/go-tmp GOPATH=/out/gopath
mkdir -p "$GOCACHE" "$GOMODCACHE" "$GOTMPDIR"
test "$(cat /sys/fs/cgroup/memory.max)" = 4294967296
test "$(cat /sys/fs/cgroup/memory.swap.max)" = 0
test "$(cat /sys/fs/cgroup/pids.max)" = 64
test "$(cat /sys/fs/cgroup/cpu.max)" = '200000 100000'
cd "/out/$variant-src"
case "$case_name" in
 line) pkg=./internal/momentline; dir=internal/momentline; pattern='^(TestEvaluateAreaMatchesThreeMomentReference)$'
  if test "$variant" = candidate; then pattern='^(TestEvaluateAreaMatchesThreeMomentReference|TestDyadicLineMomentsMatchRationalOracle|TestDyadicLineMomentResultsOwnRationals)$'; fi ;;
 api) pkg=./apitest; dir=apitest; pattern='^(TestRegionMomentsRequestedOrderControlsOverflow|TestOverflowingSecondMomentKeepsExactArea|TestLineRationalRoundingIsBounded|TestUnderflowingLineRegionAreaPublishesBoundedZero|TestOffsetPrismRectangle|TestOffsetPrismCircle|TestOffsetPrismNarrowNeckAdmitted)$' ;;
 root) pkg=.; dir=.; pattern='^(TestFirstOrderMomentsMatchFullAreaCentroidAndBounds|TestThirdOrderMomentsOfASector)$' ;;
 neck) pkg=./apitest; dir=apitest; pattern='^TestOffsetPrismNarrowNeckAdmitted$'; bench='^BenchmarkOffsetPrismNarrowNeck$' ;;
 circle) pkg=./apitest; dir=apitest; pattern='^TestOffsetPrismCircle$'; bench='^BenchmarkOffsetPrismCircle$' ;;
 *) exit 64 ;;
esac
binary="/out/$variant-$case_name.test"
case "$action" in
 compile_normal) exec go test -c -p=1 -o "$binary" "$pkg" ;;
 compile_race) exec go test -c -race -p=1 -o "/out/$variant-$case_name-race.test" "$pkg" ;;
 test|race|admission)
  if test "$action" = race; then binary="/out/$variant-$case_name-race.test"; fi
  timeout=30s
  if test "$action" = admission; then timeout=45s; fi
  cd "$dir"
  exec "$binary" -test.run "$pattern" -test.v -test.parallel=1 -test.count=1 -test.timeout="$timeout" ;;
 consumer) cd "$dir"; exec "$binary" -test.run '^$' -test.bench "$bench" -test.benchtime="${count}x" -test.benchmem -test.count=1 -test.timeout=45s ;;
 *) exit 64 ;;
esac
