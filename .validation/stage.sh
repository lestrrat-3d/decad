#!/usr/bin/env bash
set -euo pipefail
mode=${1:?stage required}
export GOMAXPROCS=2 GOFLAGS=-p=1 GOENV=off GOTOOLCHAIN=local
export GOCACHE=/out/go-cache GOMODCACHE=/out/go-mod-cache GOTMPDIR=/out/go-tmp GOPATH=/out/gopath
mkdir -p "$GOCACHE" "$GOMODCACHE" "$GOTMPDIR"
test "$(cat /sys/fs/cgroup/memory.max)" = 4294967296
test "$(cat /sys/fs/cgroup/memory.swap.max)" = 0
test "$(cat /sys/fs/cgroup/pids.max)" = 64
test "$(cat /sys/fs/cgroup/cpu.max)" = '200000 100000'
case "$mode" in
 compile_baseline|consumer_baseline|bench_baseline_[123]) variant=baseline ;;
 compile_candidate|test_candidate|consumer_candidate|bench_candidate_[123]) variant=candidate ;;
 compile_diagnostic|diagnostic_rods|diagnostic_washer) variant=diagnostic ;;
 *) exit 64 ;;
esac
check_sources() {
 local directory=$1
 case "$variant" in
 baseline)
  test "$(sha256sum "$directory/internal/meshbool/boolean_cut.go" | cut -d' ' -f1)" = c263651be70e03aa2a289b09a6fe07ce8bf8e97069dabd8b15c87d1bbe2fee08
  test "$(sha256sum "$directory/internal/meshbool/polygon_exact.go" | cut -d' ' -f1)" = 2fbd2b70ed2fadec3a6d2a69f72038444bc65afbb6238b5537f18d30a4fa3a75
  test "$(sha256sum "$directory/internal/proofbound/budget.go" | cut -d' ' -f1)" = 23e0ee55ced4fe26232151c83b264a6ee7964b20a66ee6747591ae351a6c7f57
  test "$(sha256sum "$directory/apitest/boolean_bench_test.go" | cut -d' ' -f1)" = ba4ac2305bcbdb73b92f17fe0943d2defaae3d7ffb84c540856889f20fb94d7c
  test "$(sha256sum "$directory/apitest/boolean_test.go" | cut -d' ' -f1)" = 19efbceedac39d6bd6d0a1ccd8f073d05e7e3c6879308576caf7eb7838b96d45
  test "$(sha256sum "$directory/go.mod" | cut -d' ' -f1)" = ec9f103a0e56d4fa67749bb5b77f46c194d6e47f9cd33ce479aee51569210026
  test "$(sha256sum "$directory/go.sum" | cut -d' ' -f1)" = 76866a8658a108d1d626c8261f2e4c79655b7fcc27b949d4bed18d1bcb7a46f9
  test "$(sha256sum "$directory/.golangci.yml" | cut -d' ' -f1)" = d107552570024101877e5e16d8253a199611ac7ebd45483998db5fcb070ba1c7
  test "$(sha256sum "$directory/.github/workflows/ci.yml" | cut -d' ' -f1)" = 1677610ba7cca04c8eba394f355e8e307c868b71021fd8dfc0bff587fd344c9a
  ;;
 candidate)
  test "$(sha256sum "$directory/internal/meshbool/boolean_cut.go" | cut -d' ' -f1)" = 726897b2d588104b6af024c2b267f33bb79e565bc6b64765e4866122764e0da7
  test "$(sha256sum "$directory/internal/meshbool/boolean_cut_scratch_test.go" | cut -d' ' -f1)" = df694ea5cedff174464b6701b71d87313111234834c07758036c78e0ca36d991
  test "$(sha256sum "$directory/internal/meshbool/polygon_exact.go" | cut -d' ' -f1)" = 2fbd2b70ed2fadec3a6d2a69f72038444bc65afbb6238b5537f18d30a4fa3a75
  test "$(sha256sum "$directory/internal/proofbound/budget.go" | cut -d' ' -f1)" = 23e0ee55ced4fe26232151c83b264a6ee7964b20a66ee6747591ae351a6c7f57
  test "$(sha256sum "$directory/apitest/boolean_bench_test.go" | cut -d' ' -f1)" = ba4ac2305bcbdb73b92f17fe0943d2defaae3d7ffb84c540856889f20fb94d7c
  test "$(sha256sum "$directory/apitest/boolean_test.go" | cut -d' ' -f1)" = 19efbceedac39d6bd6d0a1ccd8f073d05e7e3c6879308576caf7eb7838b96d45
  test "$(sha256sum "$directory/go.mod" | cut -d' ' -f1)" = ec9f103a0e56d4fa67749bb5b77f46c194d6e47f9cd33ce479aee51569210026
  test "$(sha256sum "$directory/go.sum" | cut -d' ' -f1)" = 76866a8658a108d1d626c8261f2e4c79655b7fcc27b949d4bed18d1bcb7a46f9
  test "$(sha256sum "$directory/.golangci.yml" | cut -d' ' -f1)" = d107552570024101877e5e16d8253a199611ac7ebd45483998db5fcb070ba1c7
  test "$(sha256sum "$directory/.github/workflows/ci.yml" | cut -d' ' -f1)" = 1677610ba7cca04c8eba394f355e8e307c868b71021fd8dfc0bff587fd344c9a
  ;;
 diagnostic)
  test "$(sha256sum "$directory/internal/meshbool/boolean_cut.go" | cut -d' ' -f1)" = 728a40ef1c2c5502beed92bbc3bc8b99d64cf4aa8ff5913b0936298a7cda8b3d
  test "$(sha256sum "$directory/internal/meshbool/boolean_cut_scratch_test.go" | cut -d' ' -f1)" = df694ea5cedff174464b6701b71d87313111234834c07758036c78e0ca36d991
  test "$(sha256sum "$directory/internal/meshbool/polygon_exact.go" | cut -d' ' -f1)" = 2fbd2b70ed2fadec3a6d2a69f72038444bc65afbb6238b5537f18d30a4fa3a75
  test "$(sha256sum "$directory/internal/proofbound/budget.go" | cut -d' ' -f1)" = 23e0ee55ced4fe26232151c83b264a6ee7964b20a66ee6747591ae351a6c7f57
  test "$(sha256sum "$directory/apitest/boolean_bench_test.go" | cut -d' ' -f1)" = e0b6bbcf7051f3ad01a396a82c39beb9d7466deb834196aa4db44f5635a4a1c7
  test "$(sha256sum "$directory/apitest/boolean_test.go" | cut -d' ' -f1)" = 86c89e7594ca3011afdae01bdce35265e82b0c70b0a8b9d61887517180d913e4
  test "$(sha256sum "$directory/go.mod" | cut -d' ' -f1)" = ec9f103a0e56d4fa67749bb5b77f46c194d6e47f9cd33ce479aee51569210026
  test "$(sha256sum "$directory/go.sum" | cut -d' ' -f1)" = 76866a8658a108d1d626c8261f2e4c79655b7fcc27b949d4bed18d1bcb7a46f9
  test "$(sha256sum "$directory/.golangci.yml" | cut -d' ' -f1)" = d107552570024101877e5e16d8253a199611ac7ebd45483998db5fcb070ba1c7
  test "$(sha256sum "$directory/.github/workflows/ci.yml" | cut -d' ' -f1)" = 1677610ba7cca04c8eba394f355e8e307c868b71021fd8dfc0bff587fd344c9a
  test "$(sha256sum "$directory/internal/meshbool/projection_scratch_diagnostic.go" | cut -d' ' -f1)" = fbb23d0f752288c4e3f21c138e49f28b88160198008508ca92a354a588ebe383
  ;;
 esac
 test "$(sha256sum "$directory/apitest/collect_p2_admission_test.go" | cut -d' ' -f1)" = e48cec312c301f7e32ed3df8fe95aefe77e2b27cefd3332b653f3bc39a827798
 test ! -e "$directory/.validation"
 test ! -L "$directory/.validation"
 if [[ "$variant" != diagnostic ]]; then
  test ! -e "$directory/internal/meshbool/projection_scratch_diagnostic.go"
 fi
}
check_sources "/src/$variant"
case "$mode" in
 compile_*)
  test ! -e "/out/$variant-src"
  mkdir "/out/$variant-src"
  cp -a "/src/$variant/." "/out/$variant-src/"
  check_sources "/out/$variant-src"
  cd "/out/$variant-src"
  if [[ "$variant" == candidate ]]; then
   go test -c -p=1 -o /out/candidate-meshbool.test ./internal/meshbool
  fi
  exec go test -c -p=1 -o "/out/$variant-apitest.test" ./apitest
  ;;
esac
check_sources "/out/$variant-src"
cd "/out/$variant-src"
case "$mode" in
 test_candidate)
  cd internal/meshbool
  exec /out/candidate-meshbool.test -test.run '^(TestRegionProjectionScratchMatchesAllocating|TestProjectionScratchRefillsAndGrows|TestRegionProjectionScratchPreservesRefusals|TestEarClipXMatchesExactTriangleOrder)$' -test.v -test.parallel=1 -test.count=1 -test.timeout=45s ;;
 consumer_*)
  cd apitest
  "/out/$variant-apitest.test" -test.run '^TestBooleanCutWasherThroughDiscKeepsStandingPost$' -test.v -test.parallel=1 -test.count=1 -test.timeout=45s
  exec "/out/$variant-apitest.test" -test.run '^$' -test.bench '^BenchmarkCollectP2CrossedRodsAdmission$' -test.benchtime=1x -test.v -test.count=1 -test.timeout=45s ;;
 bench_*)
  cd apitest
  exec "/out/$variant-apitest.test" -test.run '^$' -test.bench '^(BenchmarkBooleanUnionCrossedRods|BenchmarkCutCircularWasher)$' -test.benchtime=6x -test.benchmem -test.count=1 -test.timeout=45s ;;
 diagnostic_rods|diagnostic_washer)
  bench=BenchmarkBooleanUnionCrossedRods
  if [[ "$mode" == diagnostic_washer ]]; then bench=BenchmarkCutCircularWasher; fi
  cd apitest
  exec /out/diagnostic-apitest.test -test.run '^$' -test.bench "^${bench}$" -test.benchtime=1x -test.v -test.count=1 -test.timeout=45s ;;
 *) exit 64 ;;
esac
