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
METADATA_LIMIT = 32 << 10
STAGE_SECONDS = {'prepare_candidate': 30, 'compile_candidate': 90, 'test_candidate': 60, 'compile_race': 180, 'test_race': 60}
KILL_GRACE_SECONDS = 5
OVERALL_SECONDS = 10 * 60
SOURCE_HASHES = {
    "internal/meshbool/boolean_mesh.go": "3ae78474dc6193ea386e93eb3320d2b50cfb3baa298b41013f15bd793354986b",
    "internal/meshbool/facet_adjacency_pool.go": "5d9ab767072ed13a43da3ddd31dfac5e072a4798a0b229e0e0ab221642353aa9",
    "internal/meshbool/facet_adjacency_pool_test.go": "eb4a00a0a571dceaaa7d25f31e6b3918ee8a3087cc354a8ae610dda9884b7e69",
    "go.mod": "ec9f103a0e56d4fa67749bb5b77f46c194d6e47f9cd33ce479aee51569210026",
    "go.sum": "76866a8658a108d1d626c8261f2e4c79655b7fcc27b949d4bed18d1bcb7a46f9",
    ".golangci.yml": "d107552570024101877e5e16d8253a199611ac7ebd45483998db5fcb070ba1c7",
    ".github/workflows/ci.yml": "1677610ba7cca04c8eba394f355e8e307c868b71021fd8dfc0bff587fd344c9a"
}
CONFIG_SHA = "d107552570024101877e5e16d8253a199611ac7ebd45483998db5fcb070ba1c7"
FORBIDDEN_GALLERY = ".validation"
WORKFLOW_SHA = "ff9b6c69137f2f2f17058c03308dd2ee7665428cf6868f11fb06b12e36a605d3"
STAGE_SHA = "d228c7bd9efd87575a63a1139bc46f435e759a60f8008fc12dfb575a521b0bef"
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
    if checked(["git", "rev-parse", "HEAD^^"], cwd=checkout) != BASE:
        raise RuntimeError("candidate base differs")
    if checked(["git", "diff", "--name-only", BASE, PRODUCTION_SOURCE_COMMIT], cwd=checkout).splitlines() != ['internal/meshbool/boolean_mesh.go', 'internal/meshbool/facet_adjacency_pool.go', 'internal/meshbool/facet_adjacency_pool_test.go']:
        raise RuntimeError("candidate changed-file scope differs")
    source_sha = os.environ["DECAD_SOURCE_SHA"]
    dispatch_ref = os.environ["DECAD_DISPATCH_REF"]
    if (source_sha != os.environ["DECAD_DISPATCH_SHA"] or
            source_sha != os.environ["DECAD_WORKFLOW_SHA"] or
            not dispatch_ref.startswith("refs/heads/test-20261016-facet-owner-correctness-") or
            os.environ["DECAD_WORKFLOW_REF"] !=
            f"lestrrat-3d/decad/.github/workflows/focused-go-test.yml@{dispatch_ref}"):
        raise RuntimeError("dispatch source and workflow revision/ref differ")
    if checked(["git", "rev-parse", "HEAD^"], cwd=checkout) != PARENT_COMMIT:
        raise RuntimeError("lint validation must have the exact facet owner source as its parent")
    if checked(["git", "rev-parse", "HEAD"], cwd=checkout) != source_sha:
        raise RuntimeError("checkout does not match requested source SHA")
    changed = set(checked(["git", "diff", "--name-only", "HEAD^", "HEAD"], cwd=checkout).splitlines())
    allowed = {
        ".github/workflows/focused-go-test.yml",
        ".validation/run_hosted.py",
        ".validation/stage.sh",
    }
    if changed != allowed:
        raise RuntimeError(f"source scope differs: missing={sorted(allowed-changed)}, extra={sorted(changed-allowed)}")
    for path, expected in SOURCE_HASHES.items():
        if sha256(checkout / path) != expected:
            raise RuntimeError(f"candidate source hash mismatch: {path}")
    if sha256(checkout / ".golangci.yml") != CONFIG_SHA:
        raise RuntimeError("lint configuration hash mismatch")
    if sha256(checkout / ".github" / "workflows" / "focused-go-test.yml") != WORKFLOW_SHA:
        raise RuntimeError("temporary workflow changed")
    if sha256(checkout / ".validation/stage.sh") != STAGE_SHA:
        raise RuntimeError("temporary stage changed")


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
    actual = {}
    for prefix, directory in (("archive", source), ("copy", output / "candidate-src")):
        if prefix == "copy" and not directory.exists():
            continue
        for path, expected in SOURCE_HASHES.items():
            name = f"candidate_{prefix}_{path}"
            actual[name] = sha256(directory / path)
            if actual[name] != expected:
                raise RuntimeError(f"source hash mismatch: {name}")
        name = f"candidate_{prefix}_config"
        actual[name] = sha256(directory / ".golangci.yml")
        if actual[name] != CONFIG_SHA:
            raise RuntimeError(f"lint configuration hash mismatch: {name}")
        if os.path.lexists(directory / FORBIDDEN_GALLERY):
            raise RuntimeError(f"temporary gallery entered {prefix} source")
    return actual


def stage(mode, source, control, output, budget, report):
    if CANCELLED:
        raise RuntimeError("hosted job cancellation")
    host_headroom()
    name = f"decad-facet-owner-correctness-{uuid.uuid4().hex[:12]}"
    network = "none" if mode == "prepare_candidate" or mode.startswith("test_") else "bridge"
    uid = f"{os.getuid()}:{os.getgid()}"
    create = ["docker", "create", "--name", name, "--label", f"decad-facet-owner-correctness-run={name}",
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
                ["docker", "exec", container, "bash", "/control/stage.sh", mode], file, budget,
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
            query = ["docker", "ps", "-a", "--filter", f"label=decad-facet-owner-correctness-run={name}",
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
    if mode.startswith("test_"):
        text = log.read_text()
        passed = re.findall(r"^--- PASS: ([^ ]+)", text, re.MULTILINE)
        if not passed or not re.search(r"^PASS$", text, re.MULTILINE):
            raise RuntimeError(f"{mode}: no successful package test evidence")
        expected = {"TestFacetOwnerPoolScalarEquivalence", "TestFacetOwnerPoolBudgetAndReset", "TestFacetOwnerPoolCancellation", "TestFacetOwnerPoolConcurrentOwnership", "TestFacetAdjacencyMatchesOriginalOrderAndNilSlices", "TestFacetAdjacencyKeepsBothPhaseCancellationPolls", "TestFacetAdjacencyPreCanceledAvoidsReservation"}
        if set(passed) != expected:
            raise RuntimeError(f"{mode}: selected test evidence differs")
        record["passed_top_level_tests"] = len(passed)


def main():
    signal.signal(signal.SIGTERM, request_cancel)
    signal.signal(signal.SIGINT, request_cancel)
    if os.environ.get("GITHUB_ACTIONS") != "true" or os.environ.get("RUNNER_ENVIRONMENT") != "github-hosted":
        raise RuntimeError("this validation requires an existing GitHub-hosted runner")
    checkout = Path(__file__).resolve().parent.parent
    temporary = Path(os.environ["RUNNER_TEMP"])
    output = temporary / f"decad-facet-owner-correctness-{uuid.uuid4().hex[:12]}"
    output.mkdir(mode=0o700)
    artifacts = output / "artifacts"
    artifacts.mkdir()
    report = {"base": BASE, "production_source_commit": PRODUCTION_SOURCE_COMMIT,
              "temporary_parent_commit": PARENT_COMMIT, "stage_seconds": STAGE_SECONDS,
              "source": os.environ["DECAD_SOURCE_SHA"], "image": IMAGE,
              "source_hashes": SOURCE_HASHES, "config_sha": CONFIG_SHA,
              "workflow_file_sha256": WORKFLOW_SHA, "stage_sha256": STAGE_SHA,
              "scope": "scalar adjacency and race correctness only; no benefit measurement",
              "stages": []}
    budget = TextBudget()
    overall_deadline = time.monotonic() + OVERALL_SECONDS
    try:
        host_headroom()
        for tool in ("git", "docker"):
            if shutil.which(tool) is None:
                raise RuntimeError(f"required hosted tool is unavailable: {tool}")
        verify_source(checkout)
        source = output / "candidate"
        export_clean_source(checkout, source, PRODUCTION_SOURCE_COMMIT)
        control = checkout / ".validation"
        verify_disposable_sources(source, output)
        third_party = output / "third-party"
        third_party.mkdir()
        with (artifacts / "image-pull.log").open("wb") as log:
            code, _ = bounded_process(["docker", "pull", "--quiet", IMAGE], log, budget, seconds=60)
        if code:
            raise RuntimeError("Docker image pull failed")
        report["image_id"] = checked(["docker", "image", "inspect", "--format", "{{.Id}}", IMAGE])
        for mode in STAGE_SECONDS:
            if time.monotonic() + STAGE_SECONDS[mode] + 60 >= overall_deadline:
                raise RuntimeError("insufficient time for a stage and verified cleanup")
            stage(mode, source, control, output, budget, report)
        if CANCELLED:
            raise RuntimeError("hosted job cancellation")
        report["result"] = "bounded facet owner correctness checks passed"
    except Exception as error:
        report["result"] = "inconclusive or failed"
        report["error"] = str(error)
        raise
    finally:
        payload = json.dumps(report, indent=2).encode()
        overflow = len(payload) > METADATA_LIMIT
        if overflow:
            payload = b'{"result":"metadata overflow; inconclusive"}\n'
        (artifacts / "control.json").write_bytes(payload)
        print(f"Analytic walk quality artifacts: {artifacts}", file=sys.stderr)
        if overflow:
            raise RuntimeError("metadata exceeded 32 KiB")


if __name__ == "__main__":
    main()
