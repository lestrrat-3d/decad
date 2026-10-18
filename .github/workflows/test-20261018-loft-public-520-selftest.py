"""Bounded no-Docker failure-path checks for the temporary 520 controller."""

import importlib.util
import itertools
from pathlib import Path
import signal
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


if __name__ == "__main__":
    unittest.main()
