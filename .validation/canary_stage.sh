#!/usr/bin/env bash
set -euo pipefail
ulimit -f 1024
test "$(cat /sys/fs/cgroup/memory.max)" = 4294967296
test "$(cat /sys/fs/cgroup/memory.swap.max)" = 0
test "$(cat /sys/fs/cgroup/pids.max)" = 64
test "$(cat /sys/fs/cgroup/cpu.max)" = '200000 100000'
case "${1:?canary required}" in
  failure) exit 23 ;;
  sigterm) touch /out/canary.started; sleep 30 ;;
  *) exit 64 ;;
esac
