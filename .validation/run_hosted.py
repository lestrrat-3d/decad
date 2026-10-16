#!/usr/bin/env python3
"""Reviewed-pattern hosted thread validation. Never run locally."""
import hashlib
import json
import os
from pathlib import Path
import selectors
import re
import shutil
import signal
import subprocess
import sys
import tarfile
import time
import uuid

PRODUCTION_SOURCE_COMMIT = 'ab2857d5eb68244dac585c0756cce8baddf18a05'
CONTROL_HASHES = {'.github/workflows/focused-go-test.yml': '96828bc9ad583cfba2b7f1c9d58b22224333354180ae9239f11a2a9b5cd769d3', '.validation/stage.sh': '090d4bd07a7253c8ab0742a9806720c2b000d489e2d3c2fbb184d2c8b5d9afb8', '.validation/preview.go': '86695fbd6656bbecedeea1aebf57e6c1650db9602c94ad1750b5c70b5a04a6aa', '.validation/source.sha256': '23d7e3c2c0c122509e41362d95bd8464f80e40ca62f6b48e193c17bbdbf60786', '.validation/tools/golangci-lint-2.12.2-linux-amd64.tar.gz': '8df580d2670fed8fa984aac0507099af8df275e665215f5c7a2ae3943893a553', '.validation/tools/golangci-lint-2.12.2-checksums.txt': '9accc7943a5b4be44416a7d4efa7efb3d18c7f1919d6581cc3536e185301a2d4', '.validation/thread_overlay.go': 'b358e136723f93ee52111f4c2e44a84b2419392bb339545ec83545e3b27d7baa', '.validation/thread_diagnostic_test.go': '810c24bf0f74dda27c88aaa40e43aa10e3440e020476963be4c9b8be13bb2e33'}
TOOL_PINS = {'repository': 'golangci/golangci-lint', 'release': 'v2.12.2', 'platform': 'linux-amd64', 'archive': 'golangci-lint-2.12.2-linux-amd64.tar.gz', 'archive_bytes': 15022786, 'archive_sha256': '8df580d2670fed8fa984aac0507099af8df275e665215f5c7a2ae3943893a553', 'official_checksums_sha256': '9accc7943a5b4be44416a7d4efa7efb3d18c7f1919d6581cc3536e185301a2d4', 'binary_sha256': 'e26335d9bd381a60e5769a13b0ccc7967db5b6fb9c39a896a1f6fd0befe0a661', 'build_go': 'go1.26.2', 'source_revision': 'c0d3ddc9cf3faa61a4e378e879ece580256d76e5'}
IMAGE = "golang:1.26.8-bookworm"
MEMORY = 4 << 30
STOP_AT = 3 << 30
TEXT_LIMIT = 1 << 20
SUMMARY_LIMIT = 64 << 10
PROFILE_LIMIT = 16 << 20
METADATA_LIMIT = 32 << 10
STAGE_SECONDS = {"prepare_diagnostic": 480, "diagnostic": 180}
VALIDATION_PLANS = {"diagnostic": ("prepare_diagnostic", "diagnostic")}
PRIOR_TEST_EVIDENCE = {
    "run": "https://github.com/lestrrat-3d/decad/actions/runs/38853161515",
    "source": PRODUCTION_SOURCE_COMMIT,
    "passed_tests": ["TestCoilExternalThread", "TestCoilThreadRunsOutOfTheEnd",
                     "TestAnimatedFeatureSelection"],
    "run_result": "failed or inconclusive: lint memory threshold exceeded",
    "scope": "reference only; tests are not rerun or re-certified by this invocation",
}
KILL_GRACE_SECONDS = 5
OVERALL_SECONDS = 26 * 60
DISK_BYTES = 8 << 30
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


def stop_owned_group(proc, proof=None, hard_deadline=None, *, ignore_cancel_deadline=False):
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
    if CANCEL_DEADLINE is not None and not ignore_cancel_deadline:
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
    command_end = time.monotonic() + timeout
    if hard_deadline is not None:
        command_end = min(command_end, hard_deadline)
    # Every control command reserves its last half-second for launcher cleanup.
    deadline = command_end - 0.5
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
            code = stop_owned_group(proc, hard_deadline=command_end,
                                    ignore_cancel_deadline=allow_cancel)
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
    dispatch_ref = os.environ["DECAD_DISPATCH_REF"]
    if (source_sha != os.environ["DECAD_DISPATCH_SHA"] or
            source_sha != os.environ["DECAD_WORKFLOW_SHA"] or
            dispatch_ref != "refs/heads/test-20261016-thread-volume-diagnostic-reviewed" or
            os.environ["DECAD_WORKFLOW_REF"] !=
            f"lestrrat-3d/decad/.github/workflows/focused-go-test.yml@{dispatch_ref}"):
        raise RuntimeError("dispatch source and workflow revision/ref differ")
    if checked(["git", "rev-parse", "HEAD^"], cwd=checkout) != PRODUCTION_SOURCE_COMMIT:
        raise RuntimeError("temporary controls must directly follow the exact repair candidate")
    if checked(["git", "rev-parse", "HEAD"], cwd=checkout) != source_sha:
        raise RuntimeError("checkout does not match requested controls SHA")
    expected = {".github/workflows/focused-go-test.yml", ".validation/run_hosted.py",
                ".validation/stage.sh", ".validation/preview.go", ".validation/source.sha256",
                ".validation/tools/golangci-lint-2.12.2-linux-amd64.tar.gz",
                ".validation/tools/golangci-lint-2.12.2-checksums.txt",
                ".validation/thread_overlay.go", ".validation/thread_diagnostic_test.go"}
    changed = set(checked(["git", "diff", "--name-only", "HEAD^", "HEAD"], cwd=checkout).splitlines())
    if changed != expected:
        raise RuntimeError("temporary commit scope differs")
    for path, wanted in CONTROL_HASHES.items():
        if sha256(checkout / path) != wanted:
            raise RuntimeError(f"control hash mismatch: {path}")
    if checked(["git", "status", "--porcelain", "--untracked-files=all"], cwd=checkout):
        raise RuntimeError("temporary checkout is dirty")


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
        self.remaining = TEXT_LIMIT - 3 * SUMMARY_LIMIT

    def write(self, output, data):
        count = min(self.remaining, len(data))
        output.write(data[:count])
        self.remaining -= count
        if count != len(data):
            raise RuntimeError("total text output exceeded 1 MiB")


def cleanup_remaining(cleanup):
    if "_deadline" not in cleanup:
        cleanup["_deadline"] = time.monotonic() + KILL_GRACE_SECONDS
    if CANCEL_DEADLINE is not None and not cleanup.get("removal_budget_reserved"):
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
    # Capture cannot consume this independent removal and verification budget,
    # even when cancellation's earlier deadline has already expired.
    cleanup["_deadline"] = time.monotonic() + 3.5
    cleanup["removal_budget_reserved"] = True
    cleanup["emergency_container"] = container
    cleanup["emergency_stop_attempted"] = True
    # Leave at least 1.5 seconds for the workload launcher and verification.
    checked(["docker", "rm", "-f", container], timeout=2, allow_cancel=True,
            hard_deadline=cleanup["_deadline"])
    cleanup["emergency_rm_exit"] = 0


def bounded_process(command, output, budget, *, seconds, directory=None, container=None,
                    expected_exit=0, record=None, cleanup=None, initial_events=None):
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
                # Pause and counter reads together get at most 1.5 seconds.
                # Removal always gets its own 3.5 seconds in the following try.
                capture_deadline = time.monotonic() + 1.5
                checked(["docker", "pause", container], timeout=0.75,
                        allow_cancel=True, hard_deadline=capture_deadline)
                capture_cgroup_bounded(record, directory, initial_events, capture_deadline)
            except Exception as error:
                record["cgroup_metrics_error"] = str(error)
            try:
                remove_exact_container(container, cleanup)
            except Exception as error:
                cleanup["emergency_rm_error"] = str(error)
        try:
            reserved = bool(cleanup and cleanup.get("removal_budget_reserved"))
            reaped = stop_owned_group(proc, record.setdefault("launcher", {}),
                                      hard_deadline=cleanup["_deadline"] if reserved else None,
                                      ignore_cancel_deadline=reserved)
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
    manifest = Path(__file__).resolve().parent / "source.sha256"
    for directory in (source, output / "work"):
        if not directory.exists():
            continue
        if (directory / ".validation").exists():
            raise RuntimeError("temporary controls entered production source")
        for line in manifest.read_text().splitlines():
            wanted, name = line.split("  ", 1)
            if sha256(directory / name) != wanted:
                raise RuntimeError(f"production source changed: {name}")


def capture_cgroup_bounded(record, directory, before, deadline):
    # Read through an owned, time-bounded helper so a stalled counter read
    # cannot prevent the exact-container removal attempt.
    code = ("import json,sys; from pathlib import Path; p=Path(sys.argv[1]); "
            "print(json.dumps({'peak':int((p/'memory.peak').read_text()), "
            "'events':dict((k,int(v)) for k,v in "
            "(line.split() for line in (p/'memory.events').read_text().splitlines()))}))")
    payload = checked([sys.executable, "-c", code, str(directory)],
                      timeout=1.5, allow_cancel=True, hard_deadline=deadline)
    snapshot = json.loads(payload)
    after = snapshot["events"]
    record.update({"fresh_cgroup_peak": snapshot["peak"],
                   "oom_delta": after["oom"] - before["oom"],
                   "oom_kill_delta": after["oom_kill"] - before["oom_kill"],
                   "final_memory_events": after, "final_counters_captured": True})


def capture_cgroup(record, directory, before):
    after = events(directory)
    record.update({"fresh_cgroup_peak": int((directory / "memory.peak").read_text()),
                   "oom_delta": after["oom"] - before["oom"],
                   "oom_kill_delta": after["oom_kill"] - before["oom_kill"],
                   "final_memory_events": after})
    record["final_counters_captured"] = True


def stage(mode, source, control, output, budget, report):
    if CANCELLED:
        raise RuntimeError("hosted job cancellation")
    host_headroom()
    name = f"decad-thread-gallery-{uuid.uuid4().hex[:12]}"
    network = "bridge" if mode == "prepare_diagnostic" else "none"
    uid = f"{os.getuid()}:{os.getgid()}"
    create = ["docker", "create", "--name", name, "--label", f"decad-thread-gallery-run={name}",
              "--init", "--user", uid,
              "--memory=4g", "--memory-swap=4g", "--pids-limit=64", "--cpus=2",
              "--network", network, "--tmpfs", "/tmp:rw,size=64m",
              "--mount", f"type=bind,src={source},dst=/src,readonly",
              "--mount", f"type=bind,src={control},dst=/control,readonly",
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
        record["limits"] = limits
        record["disk_before"] = disk_usage(output)
        check_members(container, directory)
        before = events(directory)
        log = Path(report["artifacts"]) / f"{mode}.log"
        with log.open("wb") as file:
            code, _ = bounded_process(
                ["docker", "exec", container, "bash", "/control/stage.sh", mode], file, budget,
                seconds=STAGE_SECONDS[mode], directory=directory, container=container,
                expected_exit=0, record=record, cleanup=cleanup, initial_events=before)
        check_members(container, directory)
    except Exception as error:
        failure = error
        record.setdefault("primary_error", str(error))
    finally:
        if directory is not None and before is not None:
            try:
                if not record.get("final_counters_captured"):
                    capture_cgroup(record, directory, before)
            except Exception as error:
                record["cgroup_metrics_error"] = str(error)
                if failure is None:
                    failure = error
                    record["primary_error"] = str(error)
        try:
            # The abnormal path already attempted exact-ID removal before launcher cleanup.
            query = ["docker", "ps", "-a", "--filter", f"label=decad-thread-gallery-run={name}",
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
        record["disk_after"] = disk_usage(output)
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


def disk_usage(path):
    st = os.statvfs(path)
    return {"capacity_bytes": st.f_blocks * st.f_frsize,
            "available_bytes": st.f_bavail * st.f_frsize,
            "free_inodes": st.f_favail}


def prepare_disk(output, report):
    # All compiler caches, binaries, module sources and linker temporaries
    # share a fixed-size disk filesystem. Nothing on the host is cleaned.
    if disk_usage(output)["available_bytes"] < 10 << 30:
        raise RuntimeError("host scratch filesystem has less than 10 GiB free")
    image = output / "scratch.ext4"
    mount = output / "scratch"
    mount.mkdir()
    report["disk"] = {"image": str(image), "mount": str(mount), "hard_bytes": DISK_BYTES}
    with image.open("xb") as file:
        file.truncate(DISK_BYTES)
    checked(["mkfs.ext4", "-q", "-F", "-m", "0", str(image)], timeout=30)
    loop = checked(["sudo", "losetup", "--find", "--show", "--nooverlap", str(image)])
    if not re.fullmatch(r"/dev/loop[0-9]+", loop):
        raise RuntimeError("unexpected owned loop device")
    report["disk"]["loop"] = loop
    checked(["sudo", "mount", "-o", "nodev,nosuid", loop, str(mount)])
    checked(["sudo", "chown", f"{os.getuid()}:{os.getgid()}", str(mount)])
    mounted = checked(["findmnt", "--noheadings", "--output", "SOURCE,FSTYPE", "--target", str(mount)]).split()
    if mounted != [loop, "ext4"] or not os.path.ismount(mount):
        raise RuntimeError("scratch mount does not match the owned loop filesystem")
    usage = disk_usage(mount)
    if not 7 << 30 < usage["capacity_bytes"] <= DISK_BYTES:
        raise RuntimeError("scratch filesystem capacity is unexpected")
    report["disk"]["initial"] = usage
    return mount


def remove_work_files(output, report):
    image = output / "scratch.ext4"
    mount = output / "scratch"
    cleanup = {}
    report["scratch_cleanup"] = cleanup
    if os.path.ismount(mount):
        cleanup["final_usage"] = disk_usage(mount)
        checked(["sudo", "umount", str(mount)], timeout=15, allow_cancel=True)
    if os.path.ismount(mount):
        raise RuntimeError("owned scratch filesystem remains mounted")
    cleanup["unmounted"] = True
    # Recover only loops associated with this run's unique exact image path,
    # including an interrupted creation that never returned its loop ID.
    associated = checked(["sudo", "losetup", "-j", str(image)], allow_cancel=True)
    loops = []
    for line in associated.splitlines():
        loop = line.split(":", 1)[0]
        if not re.fullmatch(r"/dev/loop[0-9]+", loop) or not line.endswith(f"({image})"):
            raise RuntimeError("unexpected association for owned scratch image")
        loops.append(loop)
    if len(loops) > 1:
        raise RuntimeError("multiple loop devices refer to owned scratch image")
    for loop in loops:
        checked(["sudo", "losetup", "--detach", loop], allow_cancel=True)
    if checked(["sudo", "losetup", "-j", str(image)], allow_cancel=True):
        raise RuntimeError("owned scratch loop survived detach")
    cleanup["loops_absent"] = True
    # Detaching and deleting the owned filesystem also removes read-only Go
    # module-cache directories; no chmod or recursive delete touches the host.
    image.unlink(missing_ok=True)
    if mount.exists():
        mount.rmdir()
    if image.exists() or mount.exists():
        raise RuntimeError("owned scratch storage survived cleanup")
    cleanup["verified"] = True


def main():
    signal.signal(signal.SIGTERM, request_cancel)
    signal.signal(signal.SIGINT, request_cancel)
    if os.environ.get("GITHUB_ACTIONS") != "true" or os.environ.get("RUNNER_ENVIRONMENT") != "github-hosted":
        raise RuntimeError("this validation requires a GitHub-hosted runner")
    validation = os.environ.get("DECAD_VALIDATION")
    if validation not in VALIDATION_PLANS:
        raise RuntimeError("validation must be exactly diagnostic")
    checkout = Path(__file__).resolve().parent.parent
    output = Path(os.environ["RUNNER_TEMP"]) / f"decad-thread-gallery-{uuid.uuid4().hex[:12]}"
    output.mkdir(mode=0o700)
    artifacts = output / "artifacts"
    artifacts.mkdir()
    report = {"production_source_commit": PRODUCTION_SOURCE_COMMIT,
              "controls_commit": os.environ["DECAD_SOURCE_SHA"], "control_hashes": CONTROL_HASHES,
              "stage_seconds": STAGE_SECONDS, "image": IMAGE, "tool_pins": TOOL_PINS, "stages": [],
              "validation": validation, "selected_stages": VALIDATION_PLANS[validation],
              "prior_test_evidence": PRIOR_TEST_EVIDENCE,
              "publication_approved": False}
    budget = TextBudget()
    deadline = time.monotonic() + OVERALL_SECONDS
    try:
        host_headroom()
        verify_source(checkout)
        scratch = prepare_disk(output, report)
        report["artifacts"] = str(artifacts)
        source = scratch / "source"
        export_clean_source(checkout, source, PRODUCTION_SOURCE_COMMIT)
        verify_disposable_sources(source, scratch)
        with (artifacts / "image-pull.log").open("wb") as log:
            bounded_process(["docker", "pull", "--quiet", IMAGE], log, budget, seconds=90)
        report["image_id"] = checked(["docker", "image", "inspect", "--format", "{{.Id}}", IMAGE])
        for mode in VALIDATION_PLANS[validation]:
            seconds = STAGE_SECONDS[mode]
            if time.monotonic() + seconds + 60 >= deadline:
                raise RuntimeError("insufficient stage and cleanup time")
            stage(mode, source, checkout / ".validation", scratch, budget, report)
        lines = (artifacts / "diagnostic.log").read_text().splitlines()
        if sum(line.startswith("--- PASS: TestThreadVolumeDiagnostic (") for line in lines) != 1:
            raise RuntimeError("missing successful diagnostic test completion")
        report["result"] = "reporting-only diagnostic completed; original producer result is in diagnostic.log; publication not approved"
    except Exception as error:
        (artifacts / "thread.png").unlink(missing_ok=True)
        (artifacts / "thread-readme-320.png").unlink(missing_ok=True)
        report["result"] = "failed or inconclusive"
        report["error"] = str(error)
        raise
    finally:
        try:
            remove_work_files(output, report)
        except Exception as error:
            (artifacts / "thread.png").unlink(missing_ok=True)
            (artifacts / "thread-readme-320.png").unlink(missing_ok=True)
            report["scratch_cleanup_error"] = str(error)
            report["result"] = "failed or inconclusive"
            raise
        finally:
            payload = json.dumps(report, indent=2).encode()
            if len(payload) > METADATA_LIMIT:
                payload = b'{"result":"metadata overflow; inconclusive"}\n'
            (artifacts / "control.json").write_bytes(payload)
            print(f"Thread validation artifacts: {artifacts}", file=sys.stderr)


if __name__ == "__main__":
    main()
