#!/usr/bin/env python3
"""One hosted, bounded Dyadic line correctness and measurement run. Do not run locally."""

import hashlib
import json
import os
import re
from pathlib import Path
import selectors
import shutil
import signal
import subprocess
import sys
import tarfile
import time
import uuid


BASE = "3987ea5a84f662da2c307983e5c1f28988f6fbcc"
PRODUCTION_SOURCE_COMMIT = '7ce5921177b20584123c12f62eb75444109d5a8c'
PARENT_COMMIT = PRODUCTION_SOURCE_COMMIT
IMAGE = "golang:1.26.8-bookworm"
MEMORY = 4 << 30
STOP_AT = 3 << 30
TEXT_LIMIT = 1 << 20
METADATA_LIMIT = 64 << 10
STAGE_SECONDS = {}
STAGE_SPEC = {}
OVERLAY_HASHES = {'.github/workflows/focused-go-test.yml': '3956b469a558524b17e460e3a148d77a6018917b3f3b321fd0f55a7c3d411e46', '.validation/protocol.json': '33c0d06600dacd06ff2dd9fda51e862b2bdcf34e758c5234f87f5af0f8838953', '.validation/stage.sh': 'be9ddb47cf5bf11a432bef20cdda5dd77eea6cbc4ce5b50600b9145cb0a21982'}
KILL_GRACE_SECONDS = 5
OVERALL_SECONDS = 22 * 60
SOURCE_HASHES = {'baseline': {'internal/momentline/line.go': '1a984f1668df14bb1baeab3952411feaeaae858939c37a99a88291af6ce4b029', 'internal/momentline/line_test.go': '08a81b78f8d51a97150587be481df8a7f28812a968472c5161f29d7c64dcf956', 'internal/proof/dyadic.go': '6f99eded75005c0f3e56490e4de7eb87a46987ca8a6a7c8029d3c1a8c5ba18b9', 'internal/momentregion/segments.go': 'b8ed469fe8745dd4bb0d0007927d3f74475104cf0bb341a3e3ccd2e20c7b9371', 'internal/momentregion/dispatch.go': '9d363d1c851fdf154181e2a2eb600b14f4c589a6e2fe288615d867f7b8815241', 'internal/momentregion/third.go': '581366ce098b1bcb45a14cbcb50bab9f59cfe2ee050e297b41e9398bdfdffa4c', 'apitest/offset_bench_test.go': '124867eae9ba8a1770039cc455c54b1c16000bf499504f910b4d9ad33a9cda23', 'apitest/offset_test.go': '6389f768e58a22b45c55f51d06749c248725c68e323d4fde523dd72706f6e947', 'apitest/thicken_prism_test.go': 'c266351be718cba01db836f6a4b84186fd3b6aa8737571b67969f6ffcec9c640', 'apitest/moments_test.go': 'd6155671530390de977082a90f2a2572b5abf8f68b88d0f8e9ca3254962159e5', 'moments_internal_test.go': '1cb41b366fd46be9d6dbb5668e58ea5dce14d07b08afc0ba8eeb5dd036621fd9', 'go.mod': 'ec9f103a0e56d4fa67749bb5b77f46c194d6e47f9cd33ce479aee51569210026', 'go.sum': '76866a8658a108d1d626c8261f2e4c79655b7fcc27b949d4bed18d1bcb7a46f9'}, 'candidate': {'internal/momentline/line.go': 'ec0e07d7a14dd6efb4a3333f45d17e1dc718a2294b8ed6074b240b7c492579d8', 'internal/momentline/line_test.go': '08a81b78f8d51a97150587be481df8a7f28812a968472c5161f29d7c64dcf956', 'internal/proof/dyadic.go': '6f99eded75005c0f3e56490e4de7eb87a46987ca8a6a7c8029d3c1a8c5ba18b9', 'internal/momentregion/segments.go': 'b8ed469fe8745dd4bb0d0007927d3f74475104cf0bb341a3e3ccd2e20c7b9371', 'internal/momentregion/dispatch.go': '9d363d1c851fdf154181e2a2eb600b14f4c589a6e2fe288615d867f7b8815241', 'internal/momentregion/third.go': '581366ce098b1bcb45a14cbcb50bab9f59cfe2ee050e297b41e9398bdfdffa4c', 'apitest/offset_bench_test.go': '124867eae9ba8a1770039cc455c54b1c16000bf499504f910b4d9ad33a9cda23', 'apitest/offset_test.go': '6389f768e58a22b45c55f51d06749c248725c68e323d4fde523dd72706f6e947', 'apitest/thicken_prism_test.go': 'c266351be718cba01db836f6a4b84186fd3b6aa8737571b67969f6ffcec9c640', 'apitest/moments_test.go': 'd6155671530390de977082a90f2a2572b5abf8f68b88d0f8e9ca3254962159e5', 'moments_internal_test.go': '1cb41b366fd46be9d6dbb5668e58ea5dce14d07b08afc0ba8eeb5dd036621fd9', 'go.mod': 'ec9f103a0e56d4fa67749bb5b77f46c194d6e47f9cd33ce479aee51569210026', 'go.sum': '76866a8658a108d1d626c8261f2e4c79655b7fcc27b949d4bed18d1bcb7a46f9', 'internal/momentline/dyadic.go': '493964f938b0e395def749f2865e9ac7e78832a531c1b746d1f63a12bccac44d', 'internal/momentline/dyadic_test.go': 'a8421991aecb984dbec5f2b3808d092fc42389bd92312d03d48afce8a274ff5d'}}
WORKFLOW_SHA = '3956b469a558524b17e460e3a148d77a6018917b3f3b321fd0f55a7c3d411e46'
STAGE_SHA = 'be9ddb47cf5bf11a432bef20cdda5dd77eea6cbc4ce5b50600b9145cb0a21982'
CANCELLED = False
CANCEL_DEADLINE = None


def request_cancel(_signum, _frame):
    global CANCELLED, CANCEL_DEADLINE
    CANCELLED = True
    if CANCEL_DEADLINE is None:
        CANCEL_DEADLINE = time.monotonic() + KILL_GRACE_SECONDS


def leader_status(proc):
    # WNOWAIT keeps the group leader's PID reserved until group cleanup reaps it.
    return os.waitid(os.P_PID, proc.pid, os.WEXITED | os.WNOHANG | os.WNOWAIT)


def status_code(status):
    if status.si_code == os.CLD_EXITED:
        return status.si_status
    if status.si_code in (os.CLD_KILLED, os.CLD_DUMPED):
        return -status.si_status
    raise RuntimeError(f"unexpected launcher wait status: {status.si_code}")


def live_group_members(pgid, deadline):
    members = []
    for entry in Path("/proc").iterdir():
        if time.monotonic() >= deadline:
            raise RuntimeError("owned process-group inspection exceeded its deadline")
        if not entry.name.isdecimal():
            continue
        try:
            stat = (entry / "stat").read_text()
        except (FileNotFoundError, ProcessLookupError):
            continue
        fields = stat.rsplit(") ", 1)[-1].split()
        if len(fields) < 4:
            raise RuntimeError(f"malformed process status: {entry.name}")
        if int(fields[2]) == pgid and int(fields[3]) == pgid and fields[0] not in ("Z", "X"):
            members.append(int(entry.name))
            if len(members) > 64:
                raise RuntimeError("owned process group exceeds the 64-PID evidence limit")
    return members


def stop_owned_group(proc, proof=None, hard_deadline=None):
    # start_new_session makes the launched PID the process-group and session ID.
    pgid = proc.pid
    if pgid <= 1 or pgid == os.getpgrp():
        raise RuntimeError("refusing to signal own or invalid process group")
    if proof is not None:
        proof.update({"pgid": pgid, "leader_reaped": False, "live_after": None})

    def signal_group(number):
        try:
            os.killpg(pgid, number)
        except ProcessLookupError:
            pass

    deadline = time.monotonic() + 1
    if hard_deadline is not None:
        deadline = min(deadline, hard_deadline)
    if CANCEL_DEADLINE is not None:
        deadline = min(deadline, CANCEL_DEADLINE)
    signal_group(signal.SIGTERM)
    try:
        code = proc.wait(timeout=max(0, min(0.25, deadline - time.monotonic())))
    except subprocess.TimeoutExpired:
        signal_group(signal.SIGKILL)
        code = proc.wait(timeout=max(0, deadline - time.monotonic()))
    if proof is not None:
        proof["leader_reaped"] = True
        proof["exit"] = code
    members = live_group_members(pgid, deadline)
    if members:
        signal_group(signal.SIGKILL)
        while members and time.monotonic() < deadline:
            time.sleep(0.02)
            members = live_group_members(pgid, deadline)
    if proof is not None:
        proof["live_after"] = members
    if members:
        raise RuntimeError(f"owned process group {pgid} has live descendants: {members}")
    return code


def checked(command, *, cwd=None, timeout=15, allow_cancel=False, hard_deadline=None):
    proc = subprocess.Popen(command, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            start_new_session=True)
    selector = selectors.DefaultSelector()
    selector.register(proc.stdout, selectors.EVENT_READ)
    selector.register(proc.stderr, selectors.EVENT_READ)
    chunks = {proc.stdout: bytearray(), proc.stderr: bytearray()}
    deadline = time.monotonic() + timeout
    if hard_deadline is not None:
        deadline = min(deadline, hard_deadline - 0.5)
    failure = None
    group_failure = None
    code = None
    try:
        while selector.get_map() or leader_status(proc) is None:
            if CANCELLED and not allow_cancel:
                raise RuntimeError("hosted job cancellation")
            if time.monotonic() >= deadline:
                raise RuntimeError(f"{command[0]} exceeded its control-command time limit")
            for key, _ in selector.select(timeout=min(0.1, max(0, deadline - time.monotonic()))):
                data = os.read(key.fd, 65536)
                if data:
                    chunks[key.fileobj].extend(data)
                    if sum(map(len, chunks.values())) > 65536:
                        raise RuntimeError(f"{command[0]} control output exceeded 64 KiB")
                else:
                    selector.unregister(key.fileobj)
        if CANCELLED and not allow_cancel:
            raise RuntimeError("hosted job cancellation")
    except Exception as error:
        failure = error
    finally:
        selector.close()
        try:
            code = stop_owned_group(proc, hard_deadline=hard_deadline)
        except Exception as error:
            group_failure = error
        finally:
            proc.stdout.close()
            proc.stderr.close()
    if failure is not None:
        if group_failure is not None:
            raise RuntimeError(f"{failure}; launcher cleanup failed: {group_failure}") from failure
        raise failure
    if group_failure is not None:
        raise group_failure
    stdout = chunks[proc.stdout].decode(errors="replace")
    stderr = chunks[proc.stderr].decode(errors="replace")
    if code:
        raise RuntimeError(f"{command[0]} failed: {stderr[:2000]}")
    return stdout.strip()


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify_source(checkout):
    source_sha = os.environ["DECAD_SOURCE_SHA"]
    ref = os.environ["DECAD_DISPATCH_REF"]
    if (source_sha != os.environ["DECAD_DISPATCH_SHA"] or source_sha != os.environ["DECAD_WORKFLOW_SHA"] or
        not ref.startswith("refs/heads/test-20261016-dyadic-line-") or
        os.environ["DECAD_WORKFLOW_REF"] != f"lestrrat-3d/decad/.github/workflows/focused-go-test.yml@{ref}"):
        raise RuntimeError("dispatch identity mismatch")
    if checked(["git", "rev-parse", "HEAD"], cwd=checkout) != source_sha or checked(["git", "rev-parse", "HEAD^"], cwd=checkout) != PRODUCTION_SOURCE_COMMIT or checked(["git", "rev-parse", "HEAD^^"], cwd=checkout) != BASE:
        raise RuntimeError("source ancestry mismatch")
    if checked(["git", "diff", "--name-only", BASE, PRODUCTION_SOURCE_COMMIT], cwd=checkout).splitlines() != ["internal/momentline/dyadic.go", "internal/momentline/dyadic_test.go", "internal/momentline/line.go"]:
        raise RuntimeError("candidate scope mismatch")
    allowed = set(OVERLAY_HASHES) | {".validation/run_hosted.py"}
    changed = set(checked(["git", "diff", "--name-only", "HEAD^", "HEAD"], cwd=checkout).splitlines())
    if changed != allowed: raise RuntimeError("temporary controls scope mismatch")
    for path, digest in OVERLAY_HASHES.items():
        if sha256(checkout / path) != digest: raise RuntimeError(f"control mismatch: {path}")
    for path, digest in SOURCE_HASHES["candidate"].items():
        if sha256(checkout / path) != digest: raise RuntimeError(f"candidate mismatch: {path}")


def export_clean_source(checkout, destination, revision):
    destination.mkdir()
    proc = subprocess.Popen(["git", "archive", "--format=tar", revision], cwd=checkout, stdout=subprocess.PIPE)
    try:
        with tarfile.open(fileobj=proc.stdout, mode="r|") as archive:
            archive.extractall(destination, filter="data")
    finally:
        proc.stdout.close()
    if proc.wait(timeout=30):
        raise RuntimeError("git archive failed")


def host_headroom():
    available = next((int(line.split()[1]) * 1024 for line in Path("/proc/meminfo").read_text().splitlines()
                      if line.startswith("MemAvailable:")), 0)
    if available < 6 << 30:
        raise RuntimeError("host MemAvailable is below 6 GiB")
    if not Path("/sys/fs/cgroup/cgroup.controllers").is_file():
        raise RuntimeError("cgroup v2 is required")


def cg_value(path):
    value = path.read_text().strip()
    return None if value == "max" else int(value)


def cgroup_path(pid):
    lines = Path(f"/proc/{pid}/cgroup").read_text().splitlines()
    match = [line.split("::", 1)[1] for line in lines if line.startswith("0::")]
    if len(match) != 1:
        raise RuntimeError("container process has no single cgroup v2 membership")
    return Path("/sys/fs/cgroup") / match[0].lstrip("/")


def verify_cgroup(directory):
    root = Path("/sys/fs/cgroup")
    if not directory.is_dir() or not directory.is_relative_to(root):
        raise RuntimeError("container cgroup is not visible on the host")
    if len(str(directory.relative_to(root)).encode()) > 256:
        raise RuntimeError("cgroup path exceeds reviewed metadata bound")
    ancestors = [directory, *list(directory.parents)]
    ancestors = ancestors[:ancestors.index(root) + 1]
    for name in ("memory.max", "memory.swap.max", "pids.max", "cpu.max"):
        if not (directory / name).is_file():
            raise RuntimeError(f"container cgroup lacks {name}")
    def present_values(name):
        return [value for ancestor in ancestors if (file := ancestor / name).is_file()
                if (value := cg_value(file)) is not None]
    memory = min(present_values("memory.max"), default=None)
    swap = min(present_values("memory.swap.max"), default=None)
    pids = min(present_values("pids.max"), default=None)
    cpu = min((int(parts[0]) / int(parts[1]) for ancestor in ancestors
               if (file := ancestor / "cpu.max").is_file()
               if (parts := file.read_text().split())[0] != "max"), default=None)
    if (memory, swap, pids, cpu) != (MEMORY, 0, 64, 2):
        raise RuntimeError(f"unexpected effective cgroup limits: {(memory, swap, pids, cpu)}")
    return {"memory": memory, "swap": swap, "pids": pids, "cpu": cpu,
            "path": str(directory.relative_to(root))}


def events(directory):
    return {key: int(value) for key, value in
            (line.split() for line in (directory / "memory.events").read_text().splitlines())}


def check_members(container, directory):
    output = checked(["docker", "top", container, "-eo", "pid"], timeout=5)
    pids = [int(line.strip()) for line in output.splitlines()[1:] if line.strip().isdigit()]
    if not pids:
        raise RuntimeError("no container processes were observed")
    for pid in pids:
        try:
            member = cgroup_path(pid)
        except FileNotFoundError:
            continue  # A short-lived child exited during this observation.
        if member != directory and directory not in member.parents:
            raise RuntimeError(f"container process {pid} escaped its checked cgroup")


class TextBudget:
    def __init__(self):
        self.remaining = TEXT_LIMIT

    def write(self, output, data):
        count = min(self.remaining, len(data))
        output.write(data[:count])
        self.remaining -= count
        if count != len(data):
            raise RuntimeError("total text output exceeded 1 MiB")


def cleanup_remaining(cleanup):
    if "_deadline" not in cleanup:
        cleanup["_deadline"] = time.monotonic() + KILL_GRACE_SECONDS
    if CANCEL_DEADLINE is not None:
        cleanup["_deadline"] = min(cleanup["_deadline"], CANCEL_DEADLINE)
    remaining = cleanup["_deadline"] - time.monotonic()
    if remaining <= 0:
        raise RuntimeError("five-second container cleanup allowance expired")
    return remaining


def cleanup_checked(command, cleanup):
    remaining = cleanup_remaining(cleanup)
    return checked(command, timeout=remaining, allow_cancel=True,
                   hard_deadline=cleanup["_deadline"])


def remove_exact_container(container, cleanup):
    cleanup["emergency_container"] = container
    cleanup["emergency_stop_attempted"] = True
    cleanup_checked(["docker", "rm", "-f", container], cleanup)
    cleanup["emergency_rm_exit"] = 0


def bounded_process(command, output, budget, *, seconds, directory=None, container=None,
                    expected_exit=0, record=None, cleanup=None):
    if record is None:
        record = {}
    proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            start_new_session=True)
    selector = selectors.DefaultSelector()
    selector.register(proc.stdout, selectors.EVENT_READ)
    deadline = time.monotonic() + seconds
    peak_current = 0
    failure = None
    group_failure = None
    code = None
    try:
        while selector.get_map() or leader_status(proc) is None:
            if CANCELLED:
                raise RuntimeError("hosted job cancellation")
            if time.monotonic() > deadline:
                raise RuntimeError("wall-time limit")
            if directory is not None:
                current = int((directory / "memory.current").read_text())
                peak_current = max(peak_current, current)
                if current >= STOP_AT:
                    raise RuntimeError("3 GiB cgroup safety stop")
            for key, _ in selector.select(timeout=0.1):
                data = os.read(key.fd, 65536)
                if data:
                    budget.write(output, data)
                else:
                    selector.unregister(key.fileobj)
        status = leader_status(proc)
        if status is None:
            raise RuntimeError("launcher has no terminal status")
        code = status_code(status)
        record["exit"] = code
        if code != expected_exit:
            raise RuntimeError(f"unexpected launcher exit {code}; expected {expected_exit}")
    except Exception as error:
        failure = error
        record["primary_error"] = str(error)
    finally:
        selector.close()
        record["sampled_cgroup_peak"] = peak_current
        if failure is not None and container is not None:
            try:
                remove_exact_container(container, cleanup)
            except Exception as error:
                cleanup["emergency_rm_error"] = str(error)
        try:
            reaped = stop_owned_group(proc, record.setdefault("launcher", {}))
            if code is not None and reaped != code:
                raise RuntimeError("launcher exit changed before it was reaped")
            code = reaped
        except Exception as error:
            group_failure = error
            record["launcher_error"] = str(error)
        finally:
            proc.stdout.close()
    if failure is not None:
        raise failure
    if group_failure is not None:
        raise group_failure
    return code, peak_current


def verify_disposable_sources(source, output):
    for variant in ("baseline", "candidate"):
        for kind in ("archive", "copy"):
            directory = output / (variant if kind == "archive" else variant + "-src")
            if not directory.exists(): continue
            for path, expected in SOURCE_HASHES[variant].items():
                if sha256(directory / path) != expected:
                    raise RuntimeError(f"source mismatch: {variant}/{kind}/{path}")
            if os.path.lexists(directory / ".validation"):
                raise RuntimeError("controls entered source")
            if kind == "copy":
                for dest, origin in overlay_files(variant).items():
                    if sha256(directory / dest) != OVERLAY_HASHES[origin]:
                        raise RuntimeError(f"overlay mismatch: {dest}")

def overlay_files(variant):
    return {}


def stage(mode, source, control, output, budget, report):
    if CANCELLED:
        raise RuntimeError("hosted job cancellation")
    host_headroom()
    verify_disposable_sources(source, output)
    name = f"decad-dyadic-line-{uuid.uuid4().hex[:12]}"
    network = "bridge" if STAGE_SPEC[mode][1].startswith("compile_") else "none"
    uid = f"{os.getuid()}:{os.getgid()}"
    create = ["docker", "create", "--name", name, "--label", f"decad-dyadic-line-run={name}",
              "--init", "--user", uid,
              "--memory=4g", "--memory-swap=4g", "--pids-limit=64", "--cpus=2",
              "--network", network, "--tmpfs", "/tmp:rw,size=64m",
              "--mount", f"type=bind,src={source},dst=/src,readonly",
              "--mount", f"type=bind,src={control},dst=/control,readonly",
              "--mount", f"type=bind,src={output / 'third-party'},dst=/third-party,readonly",
              "--mount", f"type=bind,src={output},dst=/out",
              IMAGE, "sleep", "infinity"]
    container = None
    directory = None
    before = None
    code = None
    failure = None
    cleanup_failure = None
    record = {"mode": mode, "exit": None}
    cleanup = {}
    try:
        created = checked(create, timeout=15)
        if len(created) != 64 or any(ch not in "0123456789abcdef" for ch in created):
            raise RuntimeError("Docker did not return one container ID")
        container = created
        record["container"] = container
        if CANCELLED:
            raise RuntimeError("hosted job cancellation")
        checked(["docker", "start", container], timeout=15)
        pid = int(checked(["docker", "inspect", "--format", "{{.State.Pid}}", container]))
        directory = cgroup_path(pid)
        limits = verify_cgroup(directory)
        check_members(container, directory)
        before = events(directory)
        log = output / "artifacts" / f"{mode}.log"
        with log.open("wb") as file:
            code, _ = bounded_process(
                ["docker", "exec", container, "bash", "/control/stage.sh", *map(str, STAGE_SPEC[mode])], file, budget,
                seconds=STAGE_SECONDS[mode], directory=directory, container=container,
                expected_exit=0, record=record, cleanup=cleanup)
        check_members(container, directory)
    except Exception as error:
        failure = error
        record.setdefault("primary_error", str(error))
    finally:
        if directory is not None and before is not None:
            try:
                after = events(directory)
                record.update({"fresh_cgroup_peak": int((directory / "memory.peak").read_text()),
                               "limits": limits,
                               "oom_delta": after["oom"] - before["oom"],
                               "oom_kill_delta": after["oom_kill"] - before["oom_kill"]})
            except Exception as error:
                record["cgroup_metrics_error"] = str(error)
                if failure is None:
                    failure = error
                    record["primary_error"] = str(error)
        try:
            # The abnormal path already attempted exact-ID removal before launcher cleanup.
            query = ["docker", "ps", "-a", "--filter", f"label=decad-dyadic-line-run={name}",
                     "--no-trunc", "--format", "{{.ID}}"]
            found = cleanup_checked(query, cleanup).splitlines()
            if len(found) > 1 or any(len(item) != 64 or any(ch not in "0123456789abcdef" for ch in item)
                                     for item in found):
                raise RuntimeError("owned-container query returned unexpected IDs")
            if found and container is not None and found != [container]:
                raise RuntimeError("owned-container label resolved to another ID")
            if found:
                cleanup_checked(["docker", "rm", "-f", found[0]], cleanup)
            if cleanup_checked(query, cleanup):
                raise RuntimeError("container survived cleanup")
            cleanup["docker_absent"] = True
            if directory is not None:
                if directory.exists():
                    if (directory / "cgroup.procs").read_text().strip():
                        raise RuntimeError("owned container cgroup still contains processes")
                    cleanup["cgroup_state"] = "empty"
                else:
                    cleanup["cgroup_state"] = "removed"
            launcher = record.get("launcher")
            if launcher is not None and (launcher.get("leader_reaped") is not True or
                                         launcher.get("live_after") != []):
                raise RuntimeError("owned launcher group cleanup is unverified")
            cleanup["verified"] = True
        except Exception as error:
            cleanup_failure = error
            record["cleanup_error"] = str(error)
        record["cleanup"] = {key: value for key, value in cleanup.items() if not key.startswith("_")}
        try:
            verify_disposable_sources(source, output)
            record["source_hashes_verified"] = True
        except Exception as error:
            record["source_hash_error"] = str(error)
            if failure is None:
                failure = error
                record["primary_error"] = str(error)
        if failure is not None:
            record["primary_error"] = str(failure)
        report["stages"].append(record)
    if failure is not None:
        raise failure
    if cleanup_failure is not None:
        raise cleanup_failure
    if (record.get("oom_delta") is None or record.get("oom_kill_delta") is None or
            record.get("fresh_cgroup_peak") is None):
        raise RuntimeError(f"{mode}: fresh cgroup memory evidence is missing")
    if record["oom_delta"] != 0 or record["oom_kill_delta"] != 0:
        raise RuntimeError(f"{mode}: cgroup OOM event; result is inconclusive")
    if record["fresh_cgroup_peak"] >= STOP_AT:
        raise RuntimeError(f"{mode}: cgroup crossed the 3 GiB safety threshold between samples")
    if code != 0:
        raise RuntimeError(f"{mode}: stage failed")
    inspect_measurement(mode, log, report)


PHASE = os.environ.get("VALIDATION_PHASE", "")
PAIRS = ("baseline", "candidate", "candidate", "baseline", "baseline", "candidate", "candidate", "baseline", "baseline", "candidate")
PROBES = set()
CORRECTNESS_BOTH = ['TestEvaluateAreaMatchesThreeMomentReference']
CORRECTNESS_CANDIDATE = ['TestDyadicLineMomentsMatchRationalOracle', 'TestDyadicLineMomentResultsOwnRationals']


INTEGRATION_TESTS = {'apitest': ['TestRegionMomentsRequestedOrderControlsOverflow', 'TestOverflowingSecondMomentKeepsExactArea', 'TestLineRationalRoundingIsBounded', 'TestUnderflowingLineRegionAreaPublishesBoundedZero', 'TestOffsetPrismRectangle', 'TestOffsetPrismCircle', 'TestOffsetPrismNarrowNeckAdmitted'], 'root': ['TestFirstOrderMomentsMatchFullAreaCentroidAndBounds', 'TestThirdOrderMomentsOfASector']}
BENCHMARKS = {'neck': 'BenchmarkOffsetPrismNarrowNeck', 'circle': 'BenchmarkOffsetPrismCircle'}
ADMISSION_TESTS = {'neck': 'TestOffsetPrismNarrowNeckAdmitted', 'circle': 'TestOffsetPrismCircle'}

def configure_stages():
    if PHASE not in ("line_correctness", "integration_correctness", "consumer_neck", "consumer_circle"):
        raise RuntimeError("unreviewed Dyadic phase")
    def add(variant, action, case, count=1, probe=False):
        key = f"{len(STAGE_SPEC):02d}_{variant}_{action}_{case}"
        STAGE_SPEC[key] = [variant, action, case, count]
        STAGE_SECONDS[key] = 90 if action.startswith("compile_") else (60 if action in ("consumer", "admission") else 45)
        if probe: PROBES.add(key)
    if PHASE.endswith("correctness"):
        for case in (["line"] if PHASE == "line_correctness" else ["api", "root"]):
            for variant in ("baseline", "candidate"):
                add(variant, "compile_normal", case)
                add(variant, "test", case)
            add("candidate", "compile_race", case)
            add("candidate", "race", case)
        return
    case = PHASE.split("_",1)[1]
    for variant in ("baseline", "candidate"): add(variant, "compile_normal", case)
    for variant in ("baseline", "candidate"): add(variant, "admission", case)
    add("baseline", "consumer", case, 1, probe=True)
    for variant in PAIRS: add(variant, "consumer", case, "selected")


def inspect_measurement(mode, log, report):
    variant, action, case, count = STAGE_SPEC[mode]
    if action.startswith("compile_"): return
    text = log.read_text()
    row = {"variant":variant, "action":action, "case":case}
    if action == "consumer":
        pattern = r"^" + re.escape(BENCHMARKS[case]) + r"(?:-\d+)?\s+(\d+)\s+([\d.]+) ns/op\s+(\d+) B/op\s+(\d+) allocs/op$"
        matches = re.findall(pattern,text,re.MULTILINE)
        if len(matches) != 1: raise RuntimeError("missing or duplicate benchmark result")
        n,ns,allocated,allocs = matches[0]
        if int(n) != int(count) or not 0 < float(ns) < float("inf"):
            raise RuntimeError("invalid fixed-count result")
        row.update(iterations=int(n),ns=float(ns),bytes=int(allocated),allocs=int(allocs))
        if mode in PROBES:
            if row["ns"] > 20e9: raise RuntimeError("baseline feasibility exceeds fixed budget")
            report["fixed_counts"][case] = 2 if row["ns"] <= 10e9 else 1
            row["probe"] = True
    else:
        expected = ([ADMISSION_TESTS[case]] if action == "admission" else
                    CORRECTNESS_BOTH + (CORRECTNESS_CANDIDATE if variant == "candidate" else [])
                    if case == "line" else INTEGRATION_TESTS["apitest" if case == "api" else "root"])
        if not re.search(r"^PASS$",text,re.MULTILINE): raise RuntimeError("missing test success")
        for name in expected:
            if len(re.findall(r"^--- PASS: "+re.escape(name)+r" ",text,re.MULTILINE)) != 1:
                raise RuntimeError(f"missing exact test evidence: {name}")
        row["passed_tests"] = expected
    report["rows"][mode] = row


def summarize(report):
    if PHASE.endswith("correctness"):
        report["phase_gate"] = True
        return
    rows = [r for r in report["rows"].values() if r["action"] == "consumer" and not r.get("probe")]
    b = [r for r in rows if r["variant"] == "baseline"]
    c = [r for r in rows if r["variant"] == "candidate"]
    if len(b) != 5 or len(c) != 5: raise RuntimeError("five paired samples required")
    median = lambda rs,key: sorted(r[key] for r in rs)[2]
    neck = PHASE == "consumer_neck"
    time_ok = median(c,"ns") <= (0.98 if neck else 1.02)*median(b,"ns") and all(
        y["ns"] <= (1.02 if neck else 1.05)*x["ns"] for x,y in zip(b,c))
    allocation_ok = all(y["allocs"] <= x["allocs"] for x,y in zip(b,c))
    if neck:
        allocation_ok = allocation_ok and median(c,"bytes") <= 0.99*median(b,"bytes") and all(
            y["bytes"] < x["bytes"] for x,y in zip(b,c))
    else:
        allocation_ok = allocation_ok and all(y["bytes"] <= x["bytes"] for x,y in zip(b,c))
    report["medians"] = {v:{k:median(rs,k) for k in ("ns","bytes","allocs")}
                         for v,rs in (("baseline",b),("candidate",c))}
    report["paired_deltas"] = [{k:y[k]-x[k] for k in ("ns","bytes","allocs")} for x,y in zip(b,c)]
    report["time_gate"] = time_ok
    report["allocation_gate"] = allocation_ok
    report["phase_gate"] = time_ok and allocation_ok
    report["other_phases_required"] = True


def encode_report(report):
    return json.dumps(report, separators=(",", ":")).encode()


def main():
    signal.signal(signal.SIGTERM, request_cancel)
    signal.signal(signal.SIGINT, request_cancel)
    if os.environ.get("GITHUB_ACTIONS") != "true" or os.environ.get("RUNNER_ENVIRONMENT") != "github-hosted":
        raise RuntimeError("this validation requires an existing GitHub-hosted runner")
    checkout = Path(__file__).resolve().parent.parent
    temporary = Path(os.environ["RUNNER_TEMP"])
    output = temporary / f"decad-dyadic-line-{uuid.uuid4().hex[:12]}"
    output.mkdir(mode=0o700)
    artifacts = output / "artifacts"
    artifacts.mkdir()
    report = {"base": BASE, "production_source_commit": PRODUCTION_SOURCE_COMMIT,
              "temporary_parent_commit": PARENT_COMMIT, "stage_seconds": STAGE_SECONDS,
              "source": os.environ["DECAD_SOURCE_SHA"], "image": IMAGE,
              "source_hashes": SOURCE_HASHES,
              "workflow_file_sha256": WORKFLOW_SHA, "stage_sha256": STAGE_SHA,
              "scope": "exact Dyadic line correctness and operation-only Offset consumers; no synthetic benefit gate",
              "phase": PHASE, "stages": [], "rows": {}, "fixed_counts": {}, "control_hashes": OVERLAY_HASHES}
    budget = TextBudget()
    overall_deadline = time.monotonic() + OVERALL_SECONDS
    try:
        host_headroom()
        for tool in ("git", "docker"):
            if shutil.which(tool) is None:
                raise RuntimeError(f"required hosted tool is unavailable: {tool}")
        verify_source(checkout)
        sources = {}
        for variant, revision in (("baseline", BASE), ("candidate", PRODUCTION_SOURCE_COMMIT)):
            sources[variant] = output / variant
            export_clean_source(checkout, sources[variant], revision)
            copy = output / (variant + "-src")
            shutil.copytree(sources[variant], copy)
            for dest, origin in overlay_files(variant).items():
                shutil.copyfile(checkout / origin, copy / dest)
        control = checkout / ".validation"
        verify_disposable_sources(None, output)
        (output / "third-party").mkdir()
        with (artifacts / "image-pull.log").open("wb") as log:
            code, _ = bounded_process(["docker", "pull", "--quiet", IMAGE], log, budget, seconds=60)
        if code:
            raise RuntimeError("Docker image pull failed")
        report["image_id"] = checked(["docker", "image", "inspect", "--format", "{{.Id}}", IMAGE])
        for mode in STAGE_SECONDS:
            if time.monotonic() + STAGE_SECONDS[mode] + 60 >= overall_deadline:
                raise RuntimeError("insufficient time for a stage and verified cleanup")
            spec = STAGE_SPEC[mode]
            if spec[1] == "consumer" and spec[3] == "selected":
                spec[3] = report["fixed_counts"][spec[2]]
            stage(mode, sources[spec[0]], control, output, budget, report)
        if CANCELLED:
            raise RuntimeError("hosted job cancellation")
        summarize(report)
        if not report["phase_gate"]:
            raise RuntimeError("frozen phase acceptance gate failed")
        report["result"] = "phase passed; other required phases and independent evidence review remain"
    except Exception as error:
        report["result"] = "inconclusive or failed"
        report["error"] = str(error)
        raise
    finally:
        payload = encode_report(report)
        overflow = len(payload) > METADATA_LIMIT
        if overflow:
            payload = b'{"result":"metadata overflow; inconclusive"}\n'
        (artifacts / "control.json").write_bytes(payload)
        print(f"Dyadic line artifacts: {artifacts}", file=sys.stderr)
        if overflow:
            raise RuntimeError("metadata exceeded 64 KiB")


if __name__ == "__main__":
    configure_stages()
    main()
