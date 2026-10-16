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
  prepare_tool)
    test "$(uname -s)" = Linux
    test "$(uname -m)" = x86_64
    printf '%s  %s\n' '9accc7943a5b4be44416a7d4efa7efb3d18c7f1919d6581cc3536e185301a2d4' /control/tools/golangci-lint-2.12.2-checksums.txt | sha256sum -c -
    grep -Fx '8df580d2670fed8fa984aac0507099af8df275e665215f5c7a2ae3943893a553  golangci-lint-2.12.2-linux-amd64.tar.gz' /control/tools/golangci-lint-2.12.2-checksums.txt
    printf '%s  %s\n' '8df580d2670fed8fa984aac0507099af8df275e665215f5c7a2ae3943893a553' /control/tools/golangci-lint-2.12.2-linux-amd64.tar.gz | sha256sum -c -
    mkdir -p /out/tools
    tar -xzf /control/tools/golangci-lint-2.12.2-linux-amd64.tar.gz -C /out/tools golangci-lint-2.12.2-linux-amd64/golangci-lint
    printf '%s  %s\n' 'e26335d9bd381a60e5769a13b0ccc7967db5b6fb9c39a896a1f6fd0befe0a661' /out/tools/golangci-lint-2.12.2-linux-amd64/golangci-lint | sha256sum -c -
    go version -m /out/tools/golangci-lint-2.12.2-linux-amd64/golangci-lint > /out/tool-build-info.txt
    grep -E 'mod[[:space:]]+github.com/golangci/golangci-lint/v2[[:space:]]+v2.12.2[[:space:]]*$' /out/tool-build-info.txt
    grep -F 'GOARCH=amd64' /out/tool-build-info.txt
    grep -F 'GOOS=linux' /out/tool-build-info.txt
    /out/tools/golangci-lint-2.12.2-linux-amd64/golangci-lint version | tee /out/tool-version.txt
    grep -F 'version 2.12.2 ' /out/tool-version.txt
    ;;
  prepare_quality|prepare_render)
    cp -a /src /out/work
    cd /out/work
    go mod download
    cd /out/work/_gallery
    go mod download
    if [[ "$1" == prepare_render ]]; then
      go build -o /out/gallery .
      go build -o /out/preview /control/preview.go
    else
      cd /out/work
      go test -c -o /out/root.test .
    fi
    ;;
  ci_checks)
    cd /out/work
    export GOMAXPROCS=1 GOMEMLIMIT=1GiB GOGC=25
    go test . ./apitest/ -run '^TestCI' -count=1 -parallel=1 -timeout=150s -v
    ;;
  gallery_regression)
    cd /out/work
    /out/root.test -test.run '^TestCoilGalleryThreadVolume$' -test.parallel=1 -test.count=1 -test.timeout=150s -test.v
    ;;
  lint|root_vet|gallery_vet)
    # Go heap targets are subordinate to the unchanged aggregate cgroup stop.
    # Fresh stage cgroups separate lint and vet; disk caches remain shared.
    export GOMAXPROCS=1 GOMEMLIMIT=1GiB GOGC=25
    printf 'Quality memory controls: GOMAXPROCS=%s GOMEMLIMIT=%s GOGC=%s GOFLAGS=%s\n' "$GOMAXPROCS" "$GOMEMLIMIT" "$GOGC" "$GOFLAGS"
    cd /out/work
    case "$1" in
      lint)
        printf '%s  %s\n' 'e26335d9bd381a60e5769a13b0ccc7967db5b6fb9c39a896a1f6fd0befe0a661' /out/tools/golangci-lint-2.12.2-linux-amd64/golangci-lint | sha256sum -c -
        /out/tools/golangci-lint-2.12.2-linux-amd64/golangci-lint run --concurrency=1 --timeout=7m ./...
        ;;
      root_vet) go vet ./... ;;
      gallery_vet) cd /out/work/_gallery; go vet ./... ;;
    esac
    ;;
  render)
    cd /out/work/_gallery
    ulimit -f 16384
    /out/gallery -only thread -out /out/render
    /out/preview /out/render/features/thread.png /out/render/thread-readme-320.png
    ;;
  *) exit 64 ;;
esac
if [[ "$1" != prepare_tool ]]; then
  (cd /out/work && sha256sum -c /control/source.sha256) >/dev/null
fi
