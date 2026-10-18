"""Bounded no-Docker failure-path checks for the temporary 520 controller."""

import importlib.util
import itertools
import json
from pathlib import Path
import signal
import subprocess
from types import SimpleNamespace
import unittest
from unittest import mock


SOURCE = Path(__file__).with_name("test-20261018-loft-public-520.py")
SPEC = importlib.util.spec_from_file_location("loft_public_520_gate", SOURCE)
gate = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(gate)


class FailurePathTests(unittest.TestCase):
    def setUp(self):
        gate.INTERRUPTED = None
        self.sample = {"current": 0, "peak": 0, "swap": 0, "events": {"oom": 0, "oom_kill": 0}}

    def test_premarker_error_captures_only_after_idle_init(self):
        with mock.patch.object(gate, "init_is_only_process", return_value=True) as idle, \
                mock.patch.object(gate, "counters", return_value=self.sample) as counters, \
                mock.patch.object(gate, "docker") as docker:
            actual = gate.stop_and_capture("id", 10, Path("/unused"), 3, None, False, 20)
        self.assertEqual(actual, self.sample)
        idle.assert_called_once()
        counters.assert_called_once()
        docker.assert_not_called()

    def test_premarker_error_with_live_child_keeps_final_unknown(self):
        with mock.patch.object(gate, "init_is_only_process", return_value=False), \
                mock.patch.object(gate, "counters") as counters:
            actual = gate.stop_and_capture("id", 10, Path("/unused"), 3, None, False, 20)
        self.assertIsNone(actual)
        counters.assert_not_called()

    def test_timeout_and_memory_stop_send_term_before_final_capture(self):
        for reason in (gate.monitored_failure(330, 330, 0),
                       gate.monitored_failure(1, 330, gate.STOP)):
            with self.subTest(reason=reason):
                self.assertIsNotNone(reason)
                ticks = itertools.count(start=0, step=0.25)
                with mock.patch.object(gate.time, "monotonic", side_effect=lambda: next(ticks)), \
                        mock.patch.object(gate.time, "sleep"), \
                        mock.patch.object(gate, "init_is_only_process", side_effect=[False, True]), \
                        mock.patch.object(gate, "counters", return_value=self.sample) as counters, \
                        mock.patch.object(gate, "docker") as docker:
                    actual = gate.stop_and_capture("id", 10, Path("/unused"), 3, 11, True, 15)
                self.assertEqual(actual, self.sample)
                self.assertIn("kill -TERM -- -11", docker.call_args_list[0].args)
                counters.assert_called_once()

    def test_output_overflow_stops_and_caps_raw_evidence(self):
        raw = bytearray(b"1234567")
        with mock.patch.object(gate, "RAW_MAX", 8):
            self.assertFalse(gate.append_bounded(raw, b"ab"))
        self.assertEqual(raw, b"1234567a")
        with mock.patch.object(gate, "init_is_only_process", return_value=True), \
                mock.patch.object(gate, "counters", return_value=self.sample), \
                mock.patch.object(gate, "docker") as docker:
            actual = gate.stop_and_capture("id", 10, Path("/unused"), 3, 11, True,
                                           gate.time.monotonic() + 15)
        self.assertEqual(actual, self.sample)
        self.assertIn("kill -TERM -- -11", docker.call_args_list[0].args)

    def test_failed_quiescence_never_reads_or_labels_final_counters(self):
        ticks = itertools.count(start=0, step=0.5)
        with mock.patch.object(gate.time, "monotonic", side_effect=lambda: next(ticks)), \
                mock.patch.object(gate.time, "sleep"), \
                mock.patch.object(gate, "init_is_only_process", return_value=False), \
                mock.patch.object(gate, "counters") as counters, \
                mock.patch.object(gate, "docker") as docker:
            actual = gate.stop_and_capture("id", 10, Path("/unused"), 3, 11, True, 6)
        self.assertIsNone(actual)
        self.assertTrue(any("kill -KILL -- -11" in call.args for call in docker.call_args_list))
        counters.assert_not_called()

    def test_missing_process_group_after_permit_keeps_final_unknown(self):
        with mock.patch.object(gate, "counters") as counters, \
                mock.patch.object(gate, "docker") as docker:
            docker.return_value.returncode = 1
            docker.return_value.stdout = b""
            actual = gate.stop_and_capture("id", 10, Path("/unused"), 3, None, True,
                                           gate.time.monotonic() + 15)
        self.assertIsNone(actual)
        counters.assert_not_called()

    def test_signal_sets_bounded_stop_reason(self):
        gate.note_signal(signal.SIGTERM, None)
        self.assertEqual(gate.monitored_failure(1, 330), "received SIGTERM")

    def test_scratch_stats_require_two_named_bounded_tmpfs_mounts(self):
        exact = b"/tmp tmpfs 4096 262144\n/go-cache tmpfs 4096 262144\n"
        with mock.patch.object(gate, "docker", return_value=SimpleNamespace(stdout=exact)) as docker:
            gate.verify_scratch_sizes("a" * 64)
        docker.assert_called_once_with("exec", "a" * 64, "stat", "-f", "-c",
                                       "%n %T %S %b", "/tmp", "/go-cache", timeout=5)

        for bad in (b"/tmp tmpfs 4096 262145\n/go-cache tmpfs 4096 262144\n",
                    b"/tmp tmpfs 4096 262144\n/go-cache tmpfs 4096 262145\n",
                    b"/tmp tmpfs 4096 262144\n",
                    b"/go-cache tmpfs 4096 262144\n/tmp tmpfs 4096 262144\n",
                    b"/tmp ext2 4096 262144\n/go-cache tmpfs 4096 262144\n",
                    b"/tmp tmpfs 0 262144\n/go-cache tmpfs 4096 262144\n",
                    b"/tmp tmpfs -1 262144\n/go-cache tmpfs 4096 262144\n"):
            with self.subTest(output=bad), \
                    mock.patch.object(gate, "docker", return_value=SimpleNamespace(stdout=bad)):
                with self.assertRaises(gate.GateError):
                    gate.verify_scratch_sizes("a" * 64)

        with mock.patch.object(gate, "docker", side_effect=subprocess.TimeoutExpired("stat", 5)):
            with self.assertRaises(subprocess.TimeoutExpired):
                gate.verify_scratch_sizes("a" * 64)

    def mocked_gate_run(self, verify_error=None, permit_error=None):
        container_id = "a" * 64
        state = {"removed": False, "exited": False}
        calls = []
        stdout = mock.Mock()
        stdout.fileno.return_value = 17
        process = mock.Mock(stdout=stdout)
        process.poll.side_effect = lambda: 0 if state["exited"] else None

        def wait(timeout):
            state["exited"] = True
            return 0

        process.wait.side_effect = wait
        selector = mock.Mock()
        selector.select.return_value = [(SimpleNamespace(fileobj=stdout), 1)]

        def docker_call(*args, **_kwargs):
            calls.append(args)
            if args[:2] == ("inspect", container_id):
                if state["removed"]:
                    return SimpleNamespace(stdout=b"", returncode=1)
                info = [{"State": {"Pid": 999999999}, "Image": "image-id"}]
                return SimpleNamespace(stdout=json.dumps(info).encode(), returncode=0)
            if args[:2] == ("image", "inspect"):
                return SimpleNamespace(stdout=b"image-id", returncode=0)
            if args[0] == "exec":
                if args[-1] == "printf 'go\\n' > /tmp/permit" and permit_error:
                    raise permit_error
                return SimpleNamespace(stdout=b"", returncode=0)
            if args[:2] == ("rm", "-f"):
                state["removed"] = state["exited"] = True
                return SimpleNamespace(stdout=b"", returncode=0)
            raise AssertionError(args)

        with mock.patch.object(gate, "bounded_process",
                               side_effect=[b"image\n", (container_id + "\n").encode()]), \
                mock.patch.object(gate.subprocess, "Popen", return_value=process), \
                mock.patch.object(gate.os, "set_blocking"), \
                mock.patch.object(gate.os, "read", return_value=b"@@GATE_READY@@\n"), \
                mock.patch.object(gate.selectors, "DefaultSelector", return_value=selector), \
                mock.patch.object(gate, "docker", side_effect=docker_call), \
                mock.patch.object(gate, "cgroup", return_value=(Path("/no-cgroup-520"), 1)), \
                mock.patch.object(gate, "verify_limits", side_effect=verify_error), \
                mock.patch.object(gate, "stop_and_capture", return_value=self.sample) as stop:
            report = {}
            raw, failure = gate.run(SimpleNamespace(source=Path("/source")), report)
        return raw, failure, report, calls, stop, process

    def test_prepermit_refusal_aborts_idle_init_without_go_or_release(self):
        error = PermissionError(13, "Permission denied", "/proc/2731/root/tmp")
        raw, failure, report, calls, stop, process = self.mocked_gate_run(verify_error=error)
        self.assertIn("Permission denied", failure)
        self.assertIn(b"@@GATE_READY@@", raw)
        self.assertNotIn(b"@@GO_JSON_BEGIN@@", raw)
        self.assertTrue(report["cleanup_verified"])
        self.assertEqual(report["final_cgroup"], self.sample)
        self.assertFalse(stop.call_args.args[5])
        self.assertIn(("exec", "a" * 64, "sh", "-c", "printf 'abort\\n' > /tmp/permit"), calls)
        self.assertNotIn(("exec", "a" * 64, "sh", "-c", "printf 'go\\n' > /tmp/permit"), calls)
        self.assertFalse(any(call[0] == "exec" and "/tmp/release" in call[-1] for call in calls))
        self.assertLess(gate.SCRIPT.index('if [ "$permit" = abort ]'),
                        gate.SCRIPT.index("setsid bash"))
        subprocess.run(["bash", "-n", "-c", gate.SCRIPT], check=True)
        process.kill.assert_not_called()

    def test_ambiguous_permit_write_never_claims_pass_or_uses_a_fifo(self):
        error = subprocess.TimeoutExpired("permit", 15)
        raw, failure, report, calls, stop, _process = self.mocked_gate_run(permit_error=error)
        self.assertIn("permit write outcome unknown", failure)
        self.assertNotIn(b"@@GO_JSON_BEGIN@@", raw)
        self.assertNotIn("passed_tests", report)
        self.assertTrue(report["cleanup_verified"])
        self.assertTrue(stop.call_args.args[5])
        self.assertEqual([call for call in calls if call[0] == "exec"],
                         [("exec", "a" * 64, "sh", "-c", "printf 'go\\n' > /tmp/permit")])


if __name__ == "__main__":
    unittest.main()
