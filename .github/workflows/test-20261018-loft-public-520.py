"""One temporary, fail-closed hosted gate for the public Loft/prism 520 test."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import selectors
import subprocess
import sys
import time


TEST = "TestLoftContactDirectShellUnionVaryingRectangleBothOrders"
WORKFLOW = ".github/workflows/test-20261018-loft-public-520.yml"
IMAGE = "golang@sha256:26de5cf8eac40c12a1ff8de1a634971b4654fd881225eed7ace643b8c769f2f5"
SKETCH = "v0.0.0-20261018052140-60cc6aa3eea7"
SKETCH_SUM = "h1:pcuJ+W1txUXhM+w1FzzqLohkScJTyW5vU3VZDiWujFA="
SKETCH_MOD_SUM = "h1:yU8fsrfp5gVwwjxD96OCgw/2R25SW4Smuz+U7iBWzT0="
HARD = 4 * 1024**3
STOP = 3 * 1024**3
RAW_MAX = 1024**2
REPORT_MAX = 64 * 1024
OUTER_SECONDS = 360

# Fill these from the separately reviewed, clean public-route commit. The
# control cannot execute a Go workload while either value is unset.
APPROVED_TARGET_SHA = ""
APPROVED_TARGET_TREE = ""

HEX40 = re.compile(r"[0-9a-f]{40}\Z")
HEX64 = re.compile(r"[0-9a-f]{64}\Z")


class GateError(RuntimeError):
    pass


def command(argv, *, timeout=15, check=True):
    result = subprocess.run(argv, capture_output=True, timeout=timeout, check=False)
    if len(result.stdout) + len(result.stderr) > 16 * 1024:
        raise GateError(f"control command output exceeded 16 KiB: {argv[0]}")
    if check and result.returncode:
        raise GateError(f"control command failed ({result.returncode}): {argv[:3]}")
    return result


def git(path, *args):
    return command(["git", "-C", str(path), *args]).stdout.decode().strip()


def manifest(path):
    paths = bounded_process(["git", "-C", str(path), "ls-files", "-z"], 30, 2 * 1024**2)
    digest = hashlib.sha256()
    count = 0
    for raw in paths.split(b"\0"):
        if not raw:
            continue
        rel = os.fsdecode(raw)
        file = path / rel
        stat = file.lstat()
        if file.is_symlink():
            content_hash = hashlib.sha256(os.fsencode(os.readlink(file))).digest()
        else:
            content = hashlib.sha256()
            with file.open("rb") as stream:
                for chunk in iter(lambda: stream.read(1024**2), b""):
                    content.update(chunk)
            content_hash = content.digest()
        digest.update(raw + b"\0" + str(stat.st_mode & 0o777777).encode() + b"\0")
        digest.update(content_hash)
        count += 1
    return count, digest.hexdigest()


def clean(path):
    result = command(["git", "-C", str(path), "status", "--porcelain=v1",
                      "--untracked-files=all"], timeout=30)
    if result.stdout:
        raise GateError(f"checkout is dirty: {path.name}")


def pin(source):
    for module in (source, source / "_gallery"):
        mod = (module / "go.mod").read_text()
        sums = (module / "go.sum").read_text().splitlines()
        if not re.search(r"(?m)^\s*github\.com/lestrrat-3d/sketch\s+" +
                         re.escape(SKETCH) + r"\s*$", mod):
            raise GateError(f"Sketch pin differs in {module.name}")
        if re.search(r"(?m)^\s*replace\s+\(?\s*github\.com/lestrrat-3d/sketch\b", mod):
            raise GateError(f"local Sketch replacement in {module.name}")
        if re.search(r"(?m)^\s*github\.com/lestrrat-3d/sketch(?:\s+v\S+)?\s*=>", mod):
            raise GateError(f"block-form Sketch replacement in {module.name}")
        prefix = "github.com/lestrrat-3d/sketch " + SKETCH
        if prefix + " " + SKETCH_SUM not in sums or \
                prefix + "/go.mod " + SKETCH_MOD_SUM not in sums:
            raise GateError(f"Sketch checksums differ in {module.name}")


def preflight(args):
    for label, value, pattern in (("target SHA", args.target_sha, HEX40),
                                  ("target tree", args.target_tree, HEX40),
                                  ("control SHA", args.control_sha, HEX40),
                                  ("workflow hash", args.workflow_sha256, HEX64)):
        if not pattern.fullmatch(value):
            raise GateError(f"invalid {label}")
    if args.github_sha != args.control_sha or git(args.control, "rev-parse", "HEAD") != args.control_sha:
        raise GateError("run/control SHA differs from reviewed control")
    workflow_hash = hashlib.sha256((args.control / WORKFLOW).read_bytes()).hexdigest()
    if workflow_hash != args.workflow_sha256:
        raise GateError("workflow bytes differ from reviewed control")
    if git(args.source, "rev-parse", "HEAD") != args.target_sha or \
            git(args.source, "rev-parse", "HEAD^{tree}") != args.target_tree:
        raise GateError("source SHA/tree differs from dispatch inputs")
    clean(args.control)
    clean(args.source)
    pin(args.source)
    count, before = manifest(args.source)
    if not args.preflight_only:
        if args.target_sha != APPROVED_TARGET_SHA or args.target_tree != APPROVED_TARGET_TREE:
            raise GateError("target has no independently reviewed constants in this control")
        if not (args.source / "loft_contact_direct_shell_test.go").is_file():
            raise GateError("named real-producer test is missing")
    return {"control_sha": args.control_sha, "workflow_sha256": workflow_hash,
            "target_sha": args.target_sha, "target_tree": args.target_tree,
            "tracked_paths": count, "source_manifest_before": before,
            "approved_target_ready": bool(APPROVED_TARGET_SHA and APPROVED_TARGET_TREE and
                                          args.target_sha == APPROVED_TARGET_SHA and
                                          args.target_tree == APPROVED_TARGET_TREE)}


def bounded_process(argv, seconds, limit):
    proc = subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    selector = selectors.DefaultSelector()
    selector.register(proc.stdout, selectors.EVENT_READ)
    output = bytearray()
    deadline = time.monotonic() + seconds
    try:
        while proc.poll() is None or selector.get_map():
            if time.monotonic() >= deadline:
                raise GateError(f"provisioning timeout: {argv[:2]}")
            for key, _ in selector.select(timeout=0.25):
                chunk = os.read(key.fileobj.fileno(), 8192)
                if not chunk:
                    selector.unregister(key.fileobj)
                    continue
                if len(output) + len(chunk) > limit:
                    raise GateError(f"provisioning output overflow: {argv[:2]}")
                output.extend(chunk)
        if proc.wait(timeout=2):
            raise GateError(f"provisioning command failed: {argv[:2]}")
        return bytes(output)
    finally:
        selector.close()
        if proc.poll() is None:
            proc.kill()
            proc.wait(timeout=5)


def cgroup(pid, container_id):
    entries = Path(f"/proc/{pid}/cgroup").read_text().splitlines()
    if len(entries) != 1 or not entries[0].startswith("0::"):
        raise GateError("cgroup v2 is unavailable")
    name = entries[0][3:]
    if container_id[:12] not in name or ".." in Path(name).parts:
        raise GateError("container cgroup identity differs")
    path = Path("/sys/fs/cgroup") / name.lstrip("/")
    return path, path.stat().st_ino


def read_number(path):
    return int(path.read_text().strip())


def counters(path):
    events = {}
    for line in (path / "memory.events").read_text().splitlines():
        key, value = line.split()
        events[key] = int(value)
    return {"current": read_number(path / "memory.current"),
            "peak": read_number(path / "memory.peak"),
            "swap": read_number(path / "memory.swap.current"), "events": events}


def verify_limits(path, pid, container_id):
    if read_number(path / "memory.max") != HARD or \
            read_number(path / "memory.swap.max") != 0 or \
            read_number(path / "pids.max") != 64:
        raise GateError("effective memory/swap/PID limit differs")
    quota, period = (path / "cpu.max").read_text().split()
    if quota == "max" or int(quota) > 2 * int(period):
        raise GateError("effective CPU quota exceeds two cores")
    mounts = {}
    for line in Path(f"/proc/{pid}/mountinfo").read_text().splitlines():
        left, right = line.split(" - ", 1)
        fields = left.split()
        if fields[4] in ("/src", "/tmp", "/go-cache"):
            mounts[fields[4]] = (fields[5].split(","), right.split())
    if len(mounts) != 3 or "ro" not in mounts["/src"][0]:
        raise GateError("source mount is not read only")
    for name in ("/tmp", "/go-cache"):
        opts, info = mounts[name]
        stat = os.statvfs(f"/proc/{pid}/root{name}")
        if info[0] != "tmpfs" or "rw" not in opts or stat.f_blocks * stat.f_frsize > 1024**3:
            raise GateError(f"scratch mount differs: {name}")
    if read_number(path / "memory.current") >= STOP:
        raise GateError("memory reached stop threshold before Go")
    if counters(path)["swap"] or counters(path)["events"].get("oom", 0) or \
            counters(path)["events"].get("oom_kill", 0):
        raise GateError("initial swap or OOM evidence")
    if not re.fullmatch(r"[0-9a-f]{64}", container_id):
        raise GateError("captured container ID is invalid")


SCRIPT = r"""
set -eu
mkfifo /tmp/permit /tmp/release
printf '@@GATE_READY@@\n'
read -r permit < /tmp/permit
setsid bash -ceu '
  version=$(go version)
  test "$version" = "go version go1.26.8 linux/amd64"
  printf "%s\n" "$version"
  go mod download
  printf "@@GO_JSON_BEGIN@@\n"
  GOPROXY=off go test -json -count=1 -p=1 -timeout 180s \
    -run "^TestLoftContactDirectShellUnionVaryingRectangleBothOrders$" .
' &
pgid=$!
printf '@@GATE_PGID %s@@\n' "$pgid"
if wait "$pgid"; then result=0; else result=$?; fi
printf '@@GATE_DONE %s@@\n' "$result"
read -r release < /tmp/release
"""


def docker(*args, timeout=15, check=True):
    return command(["docker", *args], timeout=timeout, check=check)


def postflight(args, report):
    clean(args.control)
    clean(args.source)
    if git(args.source, "rev-parse", "HEAD") != report["target_sha"] or \
            git(args.source, "rev-parse", "HEAD^{tree}") != report["target_tree"]:
        raise GateError("source commit or tree changed")
    count, after = manifest(args.source)
    report["source_manifest_after"] = after
    if count != report["tracked_paths"] or after != report["source_manifest_before"]:
        raise GateError("source tracked manifest changed")
    if hashlib.sha256((args.control / WORKFLOW).read_bytes()).hexdigest() != report["workflow_sha256"]:
        raise GateError("control workflow changed")


def parse_go_json(data):
    marker = b"@@GO_JSON_BEGIN@@\n"
    if data.count(marker) != 1:
        raise GateError("Go JSON start marker is missing or repeated")
    text = data.split(marker, 1)[1].decode("utf-8", "strict")
    text = text.split("@@GATE_DONE ", 1)[0]
    events = {}
    package_pass = 0
    required = {TEST, TEST + "/prism_first", TEST + "/loft_first"}
    for index, line in enumerate(text.splitlines()):
        event = json.loads(line)
        action, name = event.get("Action"), event.get("Test")
        if event.get("Package") != "github.com/lestrrat-3d/decad":
            raise GateError("unexpected Go package event")
        if action in ("fail", "skip"):
            raise GateError("Go test failed or skipped")
        if name and name not in required:
            raise GateError("unexpected test or subtest")
        if name in required and action in ("run", "pass"):
            events.setdefault((name, action), []).append(index)
        if not name and action == "pass":
            package_pass += 1
            package_pass_index = index
    if any(len(events.get((name, action), [])) != 1 for name in required for action in ("run", "pass")) or \
            package_pass != 1:
        raise GateError("required ordered Go events are incomplete or duplicated")
    for name in required:
        if events[(name, "run")][0] >= events[(name, "pass")][0]:
            raise GateError("Go pass preceded its run")
    if events[(TEST, "run")][0] >= min(events[(TEST + "/" + child, "run")][0]
                                       for child in ("prism_first", "loft_first")) or \
            events[(TEST, "pass")][0] <= max(events[(TEST + "/" + child, "pass")][0]
                                               for child in ("prism_first", "loft_first")) or \
            package_pass_index <= events[(TEST, "pass")][0]:
        raise GateError("parent, subtest or package event order differs")
    return sorted(required)


def run(args, report):
    raw = bytearray()
    container_id = None
    start_proc = None
    group = None
    group_stopped = False
    cgroup_path = None
    cgroup_inode = None
    init_pid = None
    saw_ready = False
    saw_begin = False
    done_code = None
    failure = None
    began = time.monotonic()
    try:
        pull = bounded_process(["docker", "pull", "--platform", "linux/amd64", "-q", IMAGE], 90, 64 * 1024)
        report["image_pull_sha256"] = hashlib.sha256(pull).hexdigest()
        raw.extend(pull)
        create = ["docker", "create", "--platform", "linux/amd64", "--memory=4g",
                  "--memory-swap=4g", "--cpus=2", "--pids-limit=64", "--read-only",
                  "--security-opt=no-new-privileges", "--cap-drop=ALL",
                  "--tmpfs", "/tmp:rw,size=1024m", "--tmpfs", "/go-cache:rw,size=1024m",
                  "--mount", f"type=bind,src={args.source},dst=/src,readonly",
                  "-w", "/src", "-e", "TMPDIR=/tmp", "-e", "GOTMPDIR=/tmp",
                  "-e", "GOMODCACHE=/tmp/mod", "-e", "GOCACHE=/go-cache",
                  "-e", "GOMAXPROCS=2", "-e", "GOMEMLIMIT=2GiB",
                  "-e", "GOFLAGS=-mod=readonly -p=1", "-e", "GOWORK=off",
                  IMAGE, "bash", "-ceu", SCRIPT]
        output = bounded_process(create, 30, 4096).decode().strip()
        if not re.fullmatch(r"[0-9a-f]{64}", output):
            raise GateError("docker create returned no exact container ID")
        container_id = output
        report["container_id"] = container_id
        start_proc = subprocess.Popen(["docker", "start", "-a", container_id],
                                      stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        sel = selectors.DefaultSelector()
        sel.register(start_proc.stdout, selectors.EVENT_READ)
        pending = bytearray()
        deadline = time.monotonic() + OUTER_SECONDS
        stop_deadline = None
        try:
            while True:
                now = time.monotonic()
                if now >= deadline and failure is None:
                    failure = "360-second workload timeout"
                if cgroup_path is not None and done_code is None:
                    try:
                        if read_number(cgroup_path / "memory.current") >= STOP and failure is None:
                            failure = "3 GiB monitored stop"
                    except OSError:
                        failure = "cgroup disappeared during workload"
                if failure and group and not group_stopped:
                    docker("exec", container_id, "bash", "-c", f"kill -TERM -- -{group}",
                           timeout=5, check=False)
                    group_stopped = True
                    stop_deadline = now + 10
                if failure and group is None:
                    break
                if stop_deadline and now >= stop_deadline and done_code is None:
                    docker("exec", container_id, "bash", "-c", f"kill -KILL -- -{group}",
                           timeout=5, check=False)
                    break
                if done_code is not None:
                    break
                if start_proc.poll() is not None and not sel.get_map():
                    raise GateError("container init exited before final evidence")
                for key, _ in sel.select(timeout=0.5):
                    chunk = os.read(key.fileobj.fileno(), 8192)
                    if not chunk:
                        sel.unregister(key.fileobj)
                        continue
                    if len(raw) + len(chunk) > RAW_MAX:
                        failure = "1 MiB raw output limit"
                        chunk = chunk[:max(0, RAW_MAX - len(raw))]
                    raw.extend(chunk)
                    pending.extend(chunk)
                    while b"\n" in pending:
                        line, _, pending = pending.partition(b"\n")
                        if line == b"@@GATE_READY@@":
                            saw_ready = True
                        elif line.startswith(b"@@GATE_PGID "):
                            group = int(line.split()[1].removesuffix(b"@@"))
                            if group <= 1:
                                raise GateError("invalid workload process group")
                        elif line == b"@@GO_JSON_BEGIN@@":
                            saw_begin = True
                        elif line.startswith(b"@@GATE_DONE "):
                            done_code = int(line.split()[1].removesuffix(b"@@"))
                if saw_ready and cgroup_path is None:
                    info = json.loads(docker("inspect", container_id).stdout)[0]
                    init_pid = info["State"]["Pid"]
                    cgroup_path, cgroup_inode = cgroup(init_pid, container_id)
                    verify_limits(cgroup_path, init_pid, container_id)
                    if info["Image"] != docker("image", "inspect", IMAGE,
                                               "--format", "{{.Id}}").stdout.decode().strip():
                        raise GateError("container image identity differs from pinned image")
                    report["image_id"] = info["Image"]
                    docker("exec", container_id, "sh", "-c", "printf 'go\\n' > /tmp/permit")
            if cgroup_path is None or init_pid is None or not saw_begin or done_code is None:
                raise GateError("workload or cgroup evidence is incomplete")
            if cgroup_path.stat().st_ino != cgroup_inode:
                raise GateError("cgroup identity changed")
            info = json.loads(docker("inspect", container_id).stdout)[0]
            if info["State"]["Pid"] != init_pid or not info["State"]["Running"]:
                raise GateError("container init changed or stopped")
            top = docker("top", container_id, "-eo", "pid").stdout.decode().splitlines()
            if len(top) != 2 or top[0].strip() != "PID" or int(top[1]) != init_pid:
                raise GateError("Go workload or child remains alive")
            final = counters(cgroup_path)
            report["final_cgroup"] = final
            report["cgroup_inode"] = cgroup_inode
            report["init_pid"] = init_pid
            if final["peak"] >= STOP or final["swap"] or \
                    final["events"].get("oom", 0) or final["events"].get("oom_kill", 0):
                raise GateError("final cgroup memory, swap or OOM gate failed")
            if failure or done_code != 0:
                raise GateError(failure or f"Go workload exited {done_code}")
            report["passed_tests"] = parse_go_json(bytes(raw))
            postflight(args, report)
        finally:
            sel.close()
    except (GateError, OSError, ValueError, subprocess.TimeoutExpired) as exc:
        failure = str(exc)
    finally:
        if container_id:
            if cgroup_path and "final_cgroup" not in report:
                try:
                    report["final_cgroup"] = counters(cgroup_path)
                except (OSError, ValueError):
                    failure = (failure or "failure") + "; final cgroup counters unavailable"
            if start_proc and start_proc.poll() is None:
                try:
                    docker("exec", container_id, "sh", "-c", "printf 'release\\n' > /tmp/release",
                           timeout=5, check=False)
                    start_proc.wait(timeout=5)
                except (GateError, subprocess.TimeoutExpired):
                    failure = (failure or "failure") + "; clean init release failed"
            try:
                removed = docker("rm", "-f", container_id, timeout=10, check=False)
                gone = docker("inspect", container_id, check=False).returncode != 0
                report["cleanup_verified"] = removed.returncode == 0 and gone
                if not report["cleanup_verified"]:
                    failure = (failure or "failure") + "; exact container cleanup failed"
            except (GateError, subprocess.TimeoutExpired):
                failure = (failure or "failure") + "; exact container cleanup unverified"
        if start_proc and start_proc.poll() is None:
            start_proc.kill()
            start_proc.wait(timeout=5)
    report["elapsed_seconds"] = round(time.monotonic() - began, 3)
    report["raw_output_bytes"] = len(raw)
    report["raw_output_sha256"] = hashlib.sha256(raw).hexdigest()
    report["failure"] = failure
    return raw, failure


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--control", type=Path, required=True)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--report-dir", type=Path, required=True)
    parser.add_argument("--target-sha", required=True)
    parser.add_argument("--target-tree", required=True)
    parser.add_argument("--control-sha", required=True)
    parser.add_argument("--workflow-sha256", required=True)
    parser.add_argument("--github-sha", default=os.environ.get("GITHUB_SHA", ""))
    parser.add_argument("--preflight-only", action="store_true")
    args = parser.parse_args()
    args.control = args.control.resolve()
    args.source = args.source.resolve()
    report = {"run_id": os.environ.get("GITHUB_RUN_ID", "local-preflight"),
              "image": IMAGE, "test": TEST, "memory_hard": HARD,
              "go_command": ["go", "test", "-json", "-count=1", "-p=1", "-timeout", "180s",
                             "-run", "^" + TEST + "$", "."],
              "memory_stop": STOP, "cpus": 2, "pids": 64,
              "scratch_bytes": [1024**3, 1024**3], "raw_output_limit": RAW_MAX,
              "go_timeout_seconds": 180, "outer_seconds": OUTER_SECONDS,
              "provisioning_outside_cgroup": ["checkout", "image pull"]}
    raw = b""
    failure = None
    try:
        report.update(preflight(args))
        if not args.preflight_only:
            raw, failure = run(args, report)
    except (GateError, OSError, ValueError, subprocess.TimeoutExpired) as exc:
        failure = str(exc)
        report["failure"] = failure
    args.report_dir.mkdir(parents=True, exist_ok=True)
    (args.report_dir / "loft-public-520.raw.log").write_bytes(raw)
    encoded = json.dumps(report, sort_keys=True, indent=2).encode()
    if len(encoded) > REPORT_MAX:
        raise GateError("report exceeds 64 KiB")
    (args.report_dir / "loft-public-520.report.json").write_bytes(encoded)
    sys.stdout.buffer.write(raw)
    print("\nGATE_REPORT " + json.dumps(report, sort_keys=True))
    if failure:
        print("GATE_FAILED " + failure, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
