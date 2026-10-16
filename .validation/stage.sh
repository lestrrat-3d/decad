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
  prepare_diagnostic)
    cp -a /src /out/work
    cd /out/work
    go mod download
    cd /out/work/_gallery
    go mod download
    # Go's overlay reads these two pinned files without changing work/source.
    cat > /out/overlay.json <<'OVERLAY'
{"Replace":{"/out/work/_gallery/thread.go":"/control/thread_overlay.go","/out/work/_gallery/thread_diagnostic_test.go":"/control/thread_diagnostic_test.go"}}
OVERLAY
    go test -overlay=/out/overlay.json -c -o /out/thread-diagnostic.test .
    ;;
  diagnostic)
    cd /out/work/_gallery
    /out/thread-diagnostic.test -test.run '^TestThreadVolumeDiagnostic$' -test.parallel=1 -test.count=1 -test.timeout=150s -test.v
    ;;
  *) exit 64 ;;
esac
(cd /out/work && sha256sum -c /control/source.sha256) >/dev/null
