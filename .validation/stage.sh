#!/usr/bin/env bash
set -euo pipefail
mode=${1:?stage mode required}
export GOMAXPROCS=2 GOFLAGS=-p=1 GOENV=off GOTOOLCHAIN=local
export GOCACHE=/out/go-cache GOMODCACHE=/out/go-mod-cache GOTMPDIR=/out/go-tmp GOPATH=/out/gopath
export GORACE=atexit_sleep_ms=0
mkdir -p "$GOCACHE" "$GOMODCACHE" "$GOTMPDIR"
test "$(cat /sys/fs/cgroup/memory.max)" = 4294967296
test "$(cat /sys/fs/cgroup/memory.swap.max)" = 0
test "$(cat /sys/fs/cgroup/pids.max)" = 64
test "$(cat /sys/fs/cgroup/cpu.max)" = '200000 100000'
check_sources() {
  local directory=$1
  test "$(sha256sum "$directory/internal/meshbool/boolean_mesh.go" | cut -d' ' -f1)" = 3ae78474dc6193ea386e93eb3320d2b50cfb3baa298b41013f15bd793354986b
  test "$(sha256sum "$directory/internal/meshbool/facet_adjacency_pool.go" | cut -d' ' -f1)" = 5d9ab767072ed13a43da3ddd31dfac5e072a4798a0b229e0e0ab221642353aa9
  test "$(sha256sum "$directory/internal/meshbool/facet_adjacency_pool_test.go" | cut -d' ' -f1)" = eb4a00a0a571dceaaa7d25f31e6b3918ee8a3087cc354a8ae610dda9884b7e69
  test "$(sha256sum "$directory/go.mod" | cut -d' ' -f1)" = ec9f103a0e56d4fa67749bb5b77f46c194d6e47f9cd33ce479aee51569210026
  test "$(sha256sum "$directory/go.sum" | cut -d' ' -f1)" = 76866a8658a108d1d626c8261f2e4c79655b7fcc27b949d4bed18d1bcb7a46f9
  test "$(sha256sum "$directory/.golangci.yml" | cut -d' ' -f1)" = d107552570024101877e5e16d8253a199611ac7ebd45483998db5fcb070ba1c7
  test "$(sha256sum "$directory/.github/workflows/ci.yml" | cut -d' ' -f1)" = 1677610ba7cca04c8eba394f355e8e307c868b71021fd8dfc0bff587fd344c9a
  test ! -e "$directory/.validation"
  test ! -L "$directory/.validation"
}
check_sources /src
case "$mode" in
  prepare_candidate)
    mkdir -p /out/candidate-src
    cp -a /src/. /out/candidate-src/
    check_sources /out/candidate-src
    ;;
  compile_candidate)
    check_sources /out/candidate-src
    cd /out/candidate-src
    exec go test -c -p=1 -o /out/adjacency.test ./internal/meshbool
    ;;
  compile_race)
    check_sources /out/candidate-src
    cd /out/candidate-src
    exec go test -c -race -p=1 -o /out/adjacency-race.test ./internal/meshbool
    ;;
  test_candidate|test_race)
    check_sources /out/candidate-src
    cd /out/candidate-src/internal/meshbool
    binary=/out/adjacency.test
    if [[ "$mode" == test_race ]]; then binary=/out/adjacency-race.test; fi
    exec "$binary" -test.run '^(TestFacetOwnerPoolScalarEquivalence|TestFacetOwnerPoolBudgetAndReset|TestFacetOwnerPoolCancellation|TestFacetOwnerPoolConcurrentOwnership|TestFacetAdjacencyMatchesOriginalOrderAndNilSlices|TestFacetAdjacencyKeepsBothPhaseCancellationPolls|TestFacetAdjacencyPreCanceledAvoidsReservation)$' -test.v -test.parallel=1 -test.count=1 -test.timeout=45s
    ;;
  *) exit 64 ;;
esac
