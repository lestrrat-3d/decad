#!/usr/bin/env bash
set -euo pipefail
export GOMAXPROCS=2 GOFLAGS=-p=1 GOMEMLIMIT=2GiB GOGC=50
export GOCACHE=/out/go-cache GOMODCACHE=/out/go-mod-cache GOTMPDIR=/out/go-tmp
export GOPATH=/out/gopath GOENV=off GOTOOLCHAIN=local
export TMPDIR=/out/link-tmp
ulimit -f 1048576
export GOLANGCI_LINT_CACHE=/out/golangci-lint-cache
mkdir -p "$GOCACHE" "$GOMODCACHE" "$GOTMPDIR" "$GOLANGCI_LINT_CACHE" "$TMPDIR"
test "$(cat /sys/fs/cgroup/memory.max)" = 4294967296
test "$(cat /sys/fs/cgroup/memory.swap.max)" = 0
test "$(cat /sys/fs/cgroup/pids.max)" = 64
test "$(cat /sys/fs/cgroup/cpu.max)" = '200000 100000'
printf 'Compiler storage before stage %s\n' "${1:?stage required}"
df -B1 /out "$TMPDIR" "$GOTMPDIR" /tmp
df -i /out "$TMPDIR" /tmp
printf 'TMPDIR=%s GOTMPDIR=%s GOCACHE=%s GOMODCACHE=%s\n' "$TMPDIR" "$GOTMPDIR" "$GOCACHE" "$GOMODCACHE"
trap 'df -B1 /out "$TMPDIR" "$GOTMPDIR" /tmp; df -i /out "$TMPDIR" /tmp' EXIT
(cd /src && sha256sum -c /control/source.sha256) >/dev/null
case "${1:?stage required}" in
  prepare_compile)
    cp -a /src /out/work
    cd /out/work
    go mod download
    go test -c -o /out/root.test .
    cd /out/work/_gallery
    go mod download
    go test -c -o /out/gallery.test .
    go build -o /out/gallery .
    go build -o /out/preview /control/preview.go
    go install github.com/golangci/golangci-lint/v2/cmd/golangci-lint@v2.12.2
    ;;
  focused_tests)
    cd /out/work
    /out/root.test -test.run '^(TestCoilExternalThread|TestCoilThreadRunsOutOfTheEnd)$' -test.parallel=1 -test.count=1 -test.timeout=150s -test.v
    cd /out/work/_gallery
    /out/gallery.test -test.run '^TestAnimatedFeatureSelection$' -test.parallel=1 -test.count=1 -test.timeout=20s -test.v
    ;;
  lint)
    cd /out/work
    /out/gopath/bin/golangci-lint run --timeout=7m ./...
    go vet ./...
    cd /out/work/_gallery
    go vet ./...
    ;;
  render)
    cd /out/work/_gallery
    ulimit -f 16384
    /out/gallery -only thread -out /out/render
    /out/preview /out/render/features/thread.png /out/render/thread-readme-320.png
    ;;
  *) exit 64 ;;
esac
(cd /out/work && sha256sum -c /control/source.sha256) >/dev/null
