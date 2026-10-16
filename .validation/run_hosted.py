#!/usr/bin/env python3
"""One hosted, bounded facet owner production-source quality run. Do not run locally."""

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
PRODUCTION_SOURCE_COMMIT = "70e5c3e5be3dc72fcd4910df68e44c8eb131c3bd"
PARENT_COMMIT = PRODUCTION_SOURCE_COMMIT
IMAGE = "golang:1.26.8-bookworm"
MEMORY = 4 << 30
STOP_AT = 3 << 30
TEXT_LIMIT = 1 << 20
METADATA_LIMIT = 256 << 10
CGROUP_PATH_JSON_LIMIT = 4096
STAGE_SECONDS = {}
STAGE_SPEC = {}
OVERLAY_HASHES = {
    ".github/workflows/focused-go-test.yml": "8d7e3048073f2effa0fe586c10f4e753b2019027508e698934bac3e0111d2019",
    ".validation/apitest/facet_owner_census_test.go": "1fc4c0b6614e8274197d8287a7dfffc8f596f79e6cd0c3889abf778f02531100",
    ".validation/apitest/facet_owner_measure_test.go": "e6b0a4e1ec02a1fd42d45920966ffdeb6b453ca2ef9277eae18a955bb1e2b242",
    ".validation/census/baseline/boolean_mesh.go": "cbb9f0bcf352d4ca1b596cbd8496eac5a224777901662ca689ae0814392493ac",
    ".validation/census/candidate/boolean_mesh.go": "31a619ed6c5cf1fffcb55e7a74a14e0df014d8f7de9cfe34bff084193c9ba0b4",
    ".validation/helper/facet_owner_measure_test.go": "157a26ec3a0ce95764c45322af835097892989a02c92735b4115de976710a360",
    ".validation/meshbool/validation_owner_census.go": "04f8a6c4bd0439b8545793990dac2bd628cd71601831d8cb1a9182d5e89879e4",
    ".validation/protocol.json": "91e88889547b89e0dda7edc54e9ab534454766e45a82815fae9c6e972c64d9d1",
    ".validation/stage.sh": "527047b116172e7096e17543b84d07e61bfa9c5f6556395fb6ae68a9eb9c1f5a"
}
KILL_GRACE_SECONDS = 5
OVERALL_SECONDS = 30 * 60
SOURCE_HASHES = {
    "baseline": {
        "internal/meshbool/boolean_mesh.go": "b9d545b941851b85760d86fafc7242f72dae987188242edee940d35e6270d487",
        "apitest/boolean_bench_test.go": "ba4ac2305bcbdb73b92f17fe0943d2defaae3d7ffb84c540856889f20fb94d7c",
        "apitest/boolean_test.go": "19efbceedac39d6bd6d0a1ccd8f073d05e7e3c6879308576caf7eb7838b96d45",
        "apitest/prism_boolean_test.go": "383a92c83dbd30272ae55757f761c4d4bdd03417818ee4d562593f37ab58e553",
        "go.mod": "ec9f103a0e56d4fa67749bb5b77f46c194d6e47f9cd33ce479aee51569210026",
        "go.sum": "76866a8658a108d1d626c8261f2e4c79655b7fcc27b949d4bed18d1bcb7a46f9"
    },
    "candidate": {
        "internal/meshbool/boolean_mesh.go": "3ae78474dc6193ea386e93eb3320d2b50cfb3baa298b41013f15bd793354986b",
        "apitest/boolean_bench_test.go": "ba4ac2305bcbdb73b92f17fe0943d2defaae3d7ffb84c540856889f20fb94d7c",
        "apitest/boolean_test.go": "19efbceedac39d6bd6d0a1ccd8f073d05e7e3c6879308576caf7eb7838b96d45",
        "apitest/prism_boolean_test.go": "383a92c83dbd30272ae55757f761c4d4bdd03417818ee4d562593f37ab58e553",
        "go.mod": "ec9f103a0e56d4fa67749bb5b77f46c194d6e47f9cd33ce479aee51569210026",
        "go.sum": "76866a8658a108d1d626c8261f2e4c79655b7fcc27b949d4bed18d1bcb7a46f9",
        "internal/meshbool/facet_adjacency_pool.go": "5d9ab767072ed13a43da3ddd31dfac5e072a4798a0b229e0e0ab221642353aa9",
        "internal/meshbool/facet_adjacency_pool_test.go": "eb4a00a0a571dceaaa7d25f31e6b3918ee8a3087cc354a8ae610dda9884b7e69"
    }
}
CONFIG_SHA = "d107552570024101877e5e16d8253a199611ac7ebd45483998db5fcb070ba1c7"
FORBIDDEN_GALLERY = ".validation"
WORKFLOW_SHA = "8d7e3048073f2effa0fe586c10f4e753b2019027508e698934bac3e0111d2019"
STAGE_SHA = "527047b116172e7096e17543b84d07e61bfa9c5f6556395fb6ae68a9eb9c1f5a"
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
        not ref.startswith("refs/heads/test-20261016-facet-owner-measurement-") or
        os.environ["DECAD_WORKFLOW_REF"] != f"lestrrat-3d/decad/.github/workflows/focused-go-test.yml@{ref}"):
        raise RuntimeError("dispatch identity mismatch")
    if checked(["git", "rev-parse", "HEAD"], cwd=checkout) != source_sha or checked(["git", "rev-parse", "HEAD^"], cwd=checkout) != PRODUCTION_SOURCE_COMMIT or checked(["git", "rev-parse", "HEAD^^"], cwd=checkout) != BASE:
        raise RuntimeError("source ancestry mismatch")
    if checked(["git", "diff", "--name-only", BASE, PRODUCTION_SOURCE_COMMIT], cwd=checkout).splitlines() != ["internal/meshbool/boolean_mesh.go", "internal/meshbool/facet_adjacency_pool.go", "internal/meshbool/facet_adjacency_pool_test.go"]:
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
    relative_path = str(directory.relative_to(root))
    if len(json.dumps(relative_path).encode()) > CGROUP_PATH_JSON_LIMIT:
        raise RuntimeError("cgroup path exceeds the 4096-byte encoded evidence bound")
    return {"memory": memory, "swap": swap, "pids": pids, "cpu": cpu,
            "path": relative_path}


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
                if kind == "copy" and PHASE == "census" and path == "internal/meshbool/boolean_mesh.go":
                    expected = OVERLAY_HASHES[f".validation/census/{variant}/boolean_mesh.go"]
                if sha256(directory / path) != expected: raise RuntimeError(f"source mismatch: {variant}/{kind}/{path}")
            if os.path.lexists(directory / ".validation"): raise RuntimeError("controls entered source")
            if kind == "copy":
                for dest, origin in overlay_files(variant).items():
                    if sha256(directory / dest) != OVERLAY_HASHES[origin]: raise RuntimeError(f"overlay mismatch: {dest}")
                if PHASE != "census" and (directory / "internal/meshbool/validation_owner_census.go").exists():
                    raise RuntimeError("census instrument entered timing tree")

def overlay_files(variant):
    result = {"apitest/facet_owner_measure_test.go": ".validation/apitest/facet_owner_measure_test.go",
              "internal/meshbool/facet_owner_measure_test.go": ".validation/helper/facet_owner_measure_test.go"}
    if PHASE == "census":
        result.update({"apitest/facet_owner_census_test.go": ".validation/apitest/facet_owner_census_test.go",
                       "internal/meshbool/validation_owner_census.go": ".validation/meshbool/validation_owner_census.go",
                       "internal/meshbool/boolean_mesh.go": f".validation/census/{variant}/boolean_mesh.go"})
    return result


def stage(mode, source, control, output, budget, report):
    if CANCELLED:
        raise RuntimeError("hosted job cancellation")
    host_headroom()
    verify_disposable_sources(source, output)
    name = f"decad-facet-owner-measurement-{uuid.uuid4().hex[:12]}"
    network = "bridge" if STAGE_SPEC[mode][1].startswith("compile_") else "none"
    uid = f"{os.getuid()}:{os.getgid()}"
    create = ["docker", "create", "--name", name, "--label", f"decad-facet-owner-measurement-run={name}",
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
            query = ["docker", "ps", "-a", "--filter", f"label=decad-facet-owner-measurement-run={name}",
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

def configure_stages():
    allowed = {"census", "primary", "cold", "gc", "helper", "helper_0", "helper_8191", "helper_rods", "helper_washer", "heap_consumer", "heap_1", "heap_2", "heap_4", "heap_8", "cancel"}
    if PHASE not in allowed: raise RuntimeError("unreviewed measurement phase")
    helper = PHASE.startswith("helper") or PHASE in {"cancel", "heap_1", "heap_2", "heap_4", "heap_8"}
    def add(variant, action, case="rods", count=1, mode="warm", workers=1, probe=False):
        key = f"{len(STAGE_SPEC):02d}_{variant}_{action}_{case}"
        STAGE_SPEC[key] = [variant, action, case, count, mode, workers]
        STAGE_SECONDS[key] = 90 if action.startswith("compile_") else 60
        if probe: PROBES.add(key)
    helper_cases = ("alternating", "overcap")
    if PHASE in {"helper_0", "helper_8191"}:
        helper_cases = ("size_" + PHASE.split("_")[1],)
    elif PHASE in {"helper_rods", "helper_washer"}:
        census = json.loads((Path(__file__).parent / "protocol.json").read_text())["representative_census"]
        if not isinstance(census["evidence_sha256"], str) or not re.fullmatch(r"[0-9a-f]{64}", census["evidence_sha256"]):
            raise RuntimeError("reviewed census evidence must be frozen before representative timing")
        observed = census[PHASE.split("_")[1]]
        if not isinstance(observed, list) or not 1 <= len(observed) <= 32 or any(type(n) is not int or n < 0 for n in observed):
            raise RuntimeError("bounded observed census sizes must be frozen")
        positive = sorted(n for n in observed if n > 0)
        if not positive or positive[-1] > 65536:
            raise RuntimeError("representative size requires separate review")
        helper_cases = ("size_" + str(positive[(len(positive)-1)//2]),)
    for v in ("baseline", "candidate"): add(v, "compile_helper" if helper else "compile_api")
    if PHASE == "census":
        for v in ("baseline", "candidate"):
            for c in ("rods", "washer"): add(v, "admission", c); add(v, "census", c)
        return
    if PHASE in {"primary", "gc"}:
        for c in ("rods", "washer"): add("baseline", "consumer", c, 1, "warm", probe=True)
    cases = ("rods", "washer") if PHASE in {"primary", "cold", "gc", "heap_consumer"} else (helper_cases if PHASE.startswith("helper") else ("helper",))
    for case in cases:
        for v in PAIRS:
            if PHASE in {"primary", "cold", "gc"}: add(v,"consumer",case,1 if PHASE=="cold" else "selected", "warm" if PHASE=="primary" else PHASE)
            elif PHASE.startswith("helper"): add(v,"helper",case,100,case)
            elif PHASE == "heap_consumer": add(v,"heap_consumer",case)
            elif PHASE.startswith("heap_"): add(v,"heap_helper",case,1,"warm",int(PHASE.split("_")[1]))
            else: add(v,"cancel",case)
PROBES = set()

def inspect_measurement(mode, log, report):
    text = log.read_text()
    variant, action, case, count, _, workers = STAGE_SPEC[mode]
    if action.startswith("compile_"): return
    row = {"variant": variant, "action": action, "case": case, "workers": workers}
    if action in {"consumer", "helper"}:
        matches = re.findall(r"^BenchmarkValidationOwner(?:Consumer|Helper)(?:-\d+)?\s+(\d+)\s+([\d.]+) ns/op\s+(\d+) B/op\s+(\d+) allocs/op$", text, re.MULTILINE)
        if len(matches) != 1: raise RuntimeError("missing or duplicate measurement")
        n,ns,allocated,allocs=matches[0]
        if int(n)!=int(count) or float(ns)<=0: raise RuntimeError("fixed iteration mismatch")
        row.update(iterations=int(n),ns=float(ns),bytes=int(allocated),allocs=int(allocs))
        if mode in PROBES:
            if row["ns"]>20e9: raise RuntimeError("baseline feasibility exceeds fixed budget")
            report["fixed_counts"][case]=2 if row["ns"]<=10e9 else 1
            row["probe"]=True
    else:
        expected={"admission":"TestValidationConsumerAdmission","census":"TestValidationOwnerCensus","heap_consumer":"TestValidationConsumerHeap","heap_helper":"TestValidationOwnerHelperHeap","cancel":"TestValidationOwnerCancellationTiming"}[action]
        if f"--- PASS: {expected} " not in text or not re.search(r"^PASS$",text,re.MULTILINE): raise RuntimeError("missing successful test evidence")
        if action.startswith("heap_"):
            entries=[json.loads(x) for x in re.findall(r"HEAP (\{.*\})",text)]
            if [x["Phase"] for x in entries]!=["before","after_call","one_gc","two_gc","released"]: raise RuntimeError("heap lifecycle evidence missing")
            row["heap"]=entries
        elif action=="cancel":
            entries=re.findall(r"CANCEL stop=(\d+) p50_ns=(\d+) p95_ns=(\d+) max_ns=(\d+)",text)
            if [int(x[0]) for x in entries]!=[1,2,17,34,35,36,50,67]: raise RuntimeError("cancel evidence missing")
            row["cancel"]=[[int(v) for v in x] for x in entries]
            proxy=re.findall(r"RESET_PROXY p50_ns=(\d+) p95_ns=(\d+) max_ns=(\d+)",text)
            if len(proxy)!=1: raise RuntimeError("reset proxy missing")
            row["reset_proxy"]=[int(v) for v in proxy[0]]
        elif action=="census":
            entries=re.findall(r"CENSUS calls=(\d+) eligible=(\d+) overcap=(\d+) empty=(\d+) max_triangles=(\d+) histogram=\[([^]]+)\] first32=\[([^]]+)\]",text)
            if len(entries)!=1: raise RuntimeError("census evidence missing")
            values=entries[0];row["census"]=[int(x) for x in values[:5]]
            row["histogram"]=[int(x) for x in values[5].split()];row["samples"]=[int(x) for x in values[6].split()]
            calls,eligible,over,empty,_=row["census"]
            if not calls or calls!=eligible+over+empty or len(row["histogram"])!=16 or sum(row["histogram"])!=calls or len(row["samples"])!=32: raise RuntimeError("inconsistent census")
    report["rows"][mode]=row

def summarize(report):
    rows=[r for r in report["rows"].values() if not r.get("probe")]
    if PHASE in {"primary","cold","gc"} or PHASE.startswith("helper"):
        result={}
        for case in sorted({r["case"] for r in rows}):
            b=[r for r in rows if r["case"]==case and r["variant"]=="baseline"]
            c=[r for r in rows if r["case"]==case and r["variant"]=="candidate"]
            if len(b)!=5 or len(c)!=5: raise RuntimeError("five pairs missing")
            med=lambda rs,k:sorted(r[k] for r in rs)[2]
            summary={v:{k:med(rs,k) for k in ("ns","bytes","allocs")} for v,rs in (("baseline",b),("candidate",c))}
            timeok=med(c,"ns")<=(1.02 if PHASE=="primary" else 1.05)*med(b,"ns") and all(y["ns"]<=(1.05 if PHASE=="primary" else 1.10)*x["ns"] for x,y in zip(b,c))
            allocationok=med(c,"bytes")<=0.995*med(b,"bytes") and all(y["bytes"]<x["bytes"] and y["allocs"]<=x["allocs"] for x,y in zip(b,c))
            result[case]={"medians":summary,"time_gate":timeok,"allocation_gate":allocationok,"pass":timeok and (allocationok if PHASE=="primary" else True)}
        report["gates"]=result;report["phase_gate"]=all(x["pass"] for x in result.values())
    if PHASE.startswith("heap_"):
        report["retained_heap_review_required"]=True
        fields=("HeapAlloc","HeapInuse","HeapObjects","TotalAlloc","Mallocs","PauseTotalNs","NumGC")
        summaries={}
        for case in sorted({r["case"] for r in rows}):
            summaries[case]={}
            for variant in ("baseline","candidate"):
                selected=[r for r in rows if r["case"]==case and r["variant"]==variant]
                if len(selected)!=5: raise RuntimeError("five heap repetitions missing")
                summaries[case][variant]={}
                for index,phase in enumerate(("before","after_call","one_gc","two_gc","released")):
                    summaries[case][variant][phase]={}
                    for field in fields:
                        values=[r["heap"][index][field]-r["heap"][0][field] for r in selected]
                        summaries[case][variant][phase][field]={"min":min(values),"median":sorted(values)[2],"max":max(values)}
        report["heap_delta_summaries"]=summaries
        paired = {}
        for case in sorted({r["case"] for r in rows}):
            b = [r for r in rows if r["case"] == case and r["variant"] == "baseline"]
            c = [r for r in rows if r["case"] == case and r["variant"] == "candidate"]
            workers = b[0]["workers"]
            if any(r["workers"] != workers for r in b+c) or workers not in (1,2,4,8):
                raise RuntimeError("heap worker counts differ")
            paired[case] = {"workers": workers, "phases": {}}
            for index, phase in enumerate(("before","after_call","one_gc","two_gc","released")):
                paired[case]["phases"][phase] = {}
                for field in ("HeapAlloc", "HeapInuse", "TotalAlloc"):
                    totals = [(y["heap"][index][field]-y["heap"][0][field]) -
                              (x["heap"][index][field]-x["heap"][0][field]) for x,y in zip(b,c)]
                    stats = lambda values: {"pairs": values, "min": min(values), "median": sorted(values)[2], "max": max(values)}
                    paired[case]["phases"][phase][field] = {
                        "extra_bytes_total": stats(totals),
                        "extra_bytes_per_worker": stats([v/workers for v in totals])}
        report["heap_paired_extra_bytes"] = paired
    if PHASE=="cancel":
        b=[r for r in rows if r["variant"]=="baseline"];c=[r for r in rows if r["variant"]=="candidate"]
        if len(b)!=5 or len(c)!=5: raise RuntimeError("cancel pairs missing")
        report["cancel_tradeoff_review_required"]=any(y[2]-x[2]>1e6 or y[3]-x[3]>5e6 for br,cr in zip(b,c) for x,y in zip(br["cancel"],cr["cancel"]))
    if PHASE=="census":
        for case in ("rods","washer"):
            rs=[r for r in rows if r["action"]=="census" and r["case"]==case]
            if len(rs)!=2 or any(rs[0][key]!=rs[1][key] for key in ("census","histogram","samples")): raise RuntimeError("consumer census differs between sources")


def encode_report(report):
    # Only the wire representation is compacted. Shared columns preserve every
    # raw value and summary statistic without repeating their keys per row.
    wire = dict(report)
    if "heap_delta_summaries" in report:
        columns = ["Phase", "HeapAlloc", "HeapInuse", "HeapObjects", "TotalAlloc",
                   "Mallocs", "PauseTotalNs", "NumGC", "GCCPUFraction"]
        wire["heap_encoding"] = {
            "version": 1, "snapshot_columns": columns,
            "delta_summary_columns": ["min", "median", "max"],
            "paired_summary_columns": ["pairs", "min", "median", "max"]}
        wire["rows"] = {key: {**row, "heap": [[snapshot[k] for k in columns]
                          for snapshot in row["heap"]]} if "heap" in row else row
                        for key, row in report["rows"].items()}
        def pack(value, fields):
            if isinstance(value, dict):
                if set(value) == set(fields):
                    return [value[k] for k in fields]
                return {k: pack(v, fields) for k, v in value.items()}
            return value
        wire["heap_delta_summaries"] = pack(report["heap_delta_summaries"], ("min", "median", "max"))
        wire["heap_paired_extra_bytes"] = pack(report["heap_paired_extra_bytes"], ("pairs", "min", "median", "max"))
    return json.dumps(wire, separators=(",", ":")).encode()


def main():
    signal.signal(signal.SIGTERM, request_cancel)
    signal.signal(signal.SIGINT, request_cancel)
    if os.environ.get("GITHUB_ACTIONS") != "true" or os.environ.get("RUNNER_ENVIRONMENT") != "github-hosted":
        raise RuntimeError("this validation requires an existing GitHub-hosted runner")
    checkout = Path(__file__).resolve().parent.parent
    temporary = Path(os.environ["RUNNER_TEMP"])
    output = temporary / f"decad-facet-owner-measurement-{uuid.uuid4().hex[:12]}"
    output.mkdir(mode=0o700)
    artifacts = output / "artifacts"
    artifacts.mkdir()
    report = {"base": BASE, "production_source_commit": PRODUCTION_SOURCE_COMMIT,
              "temporary_parent_commit": PARENT_COMMIT, "stage_seconds": STAGE_SECONDS,
              "source": os.environ["DECAD_SOURCE_SHA"], "image": IMAGE,
              "source_hashes": SOURCE_HASHES,
              "workflow_file_sha256": WORKFLOW_SHA, "stage_sha256": STAGE_SHA,
              "scope": "separate census, fixed-count real consumers, special timing, cancellation and heap phases",
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
        report["result"] = "measurement stages complete; inspect gates and heap tradeoffs"
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
        print(f"Facet owner measurement artifacts: {artifacts}", file=sys.stderr)
        if overflow:
            raise RuntimeError("metadata exceeded 256 KiB")


if __name__ == "__main__":
    configure_stages()
    main()
