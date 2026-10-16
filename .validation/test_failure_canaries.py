"""Bounded mock canaries. Never invokes Docker, Go, network or real signals."""
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

import validation_controller as ctrl


class CleanupCanaries(unittest.TestCase):
    def setUp(self):
        ctrl.life.CANCELLED = False
        ctrl.life.CANCEL_DEADLINE = None
        self.tmp = tempfile.TemporaryDirectory(dir=Path(__file__).parent, prefix='mock-canary-')
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.artifacts = self.root / 'artifacts'
        self.artifacts.mkdir()
        self.report = {'stages': []}
        self.calls = []
        self.phase = {'name': '01-real-consumer', 'wall_seconds': 5}
        self.peak = 1024
        self.failure = None
        self.remove_failure = False
        self.capture_failure = False
        self.wait_failure = False
        self.cancel = False

    def checked(self, command, **kwargs):
        self.calls.append((command, kwargs))
        if command[:2] == ['docker', 'create']:
            # The ownership record must exist even when Docker creation times out.
            record = json.loads((self.artifacts / '01-real-consumer.json').read_text())
            self.assertIn(record['container_name'], command)
            mounts = [command[i + 1] for i, value in enumerate(command) if value == '--mount']
            self.assertTrue(any('dst=/control,readonly' in value for value in mounts))
            self.assertTrue(any('dst=/src,readonly' in value for value in mounts))
            self.assertTrue(any(value.endswith('dst=/out') for value in mounts))
            if self.failure == 'create':
                raise RuntimeError('create timeout')
            return 'a' * 64
        if command[:2] == ['docker', 'inspect']:
            self.assertLessEqual(kwargs['timeout'], 2)
            if self.failure == 'inspect':
                raise RuntimeError('inspect unavailable')
            return '12345'
        return ''

    def bounded(self, *_args, **kwargs):
        if self.cancel:
            ctrl.life.request_cancel(None, None)
        if self.wait_failure:
            raise subprocess.TimeoutExpired('owned launcher wait', 0.2)
        kwargs['record']['launcher'] = {'leader_reaped': True, 'live_after': []}
        kwargs['record']['exit'] = 0
        return 0, self.peak

    def capture(self, record, _directory, _before, deadline):
        self.assertLessEqual(deadline - ctrl.time.monotonic(), 1.5)
        if self.capture_failure:
            raise RuntimeError('counter read unavailable')
        record.update(final_counters_captured=True, fresh_cgroup_peak=self.peak,
                      oom_delta=0, oom_kill_delta=0, final_memory_events={'oom': 0, 'oom_kill': 0})

    def remove(self, target, cleanup):
        self.calls.append((['remove-exact', target], {}))
        cleanup['_deadline'] = ctrl.time.monotonic() + 3.5
        cleanup['removal_budget_reserved'] = True
        if self.remove_failure:
            raise RuntimeError('removal timeout')

    def cleanup_query(self, command, cleanup):
        self.calls.append((command, {}))
        self.assertTrue(cleanup['removal_budget_reserved'])
        if self.remove_failure:
            raise RuntimeError('removal cannot be verified')
        return 'removed' if command[0] != 'docker' else ''

    def run_mock(self, expected=None):
        with patch.object(ctrl.life, 'host_headroom'), \
                patch.object(ctrl.life, 'checked', side_effect=self.checked), \
                patch.object(ctrl.life, 'cgroup_path', return_value=Path('/owned/mock')), \
                patch.object(ctrl.life, 'verify_cgroup', return_value={}), \
                patch.object(ctrl.life, 'check_members'), \
                patch.object(ctrl.life, 'events', return_value={'oom': 0, 'oom_kill': 0}), \
                patch.object(ctrl.life, 'bounded_process', side_effect=self.bounded), \
                patch.object(ctrl.life, 'capture_cgroup_bounded', side_effect=self.capture), \
                patch.object(ctrl.life, 'remove_exact_container', side_effect=self.remove), \
                patch.object(ctrl.life, 'cleanup_checked', side_effect=self.cleanup_query):
            if expected:
                with self.assertRaisesRegex(Exception, expected):
                    ctrl.run_stage(self.phase, self.root / 'source', self.root / 'control',
                                   self.root / 'scratch', 'sha256:' + 'b' * 64,
                                   self.artifacts, ctrl.life.TextBudget(), self.report)
            else:
                ctrl.run_stage(self.phase, self.root / 'source', self.root / 'control',
                               self.root / 'scratch', 'sha256:' + 'b' * 64,
                               self.artifacts, ctrl.life.TextBudget(), self.report)
        self.assertTrue(any(command[0] == 'remove-exact' for command, _ in self.calls))
        record = json.loads((self.artifacts / '01-real-consumer.json').read_text())
        self.assertEqual(record['success'], expected is None)
        self.assertTrue((self.artifacts / 'report.json').is_file())
        return record

    def test_success_removes_exact_container_and_records_evidence(self):
        self.run_mock()

    def test_final_peak_at_threshold_fails_even_when_sampler_missed_it(self):
        self.peak = ctrl.life.STOP_AT
        self.run_mock('final memory.peak')

    def test_final_peak_above_threshold_fails(self):
        self.peak = ctrl.life.STOP_AT + 1
        self.run_mock('final memory.peak')

    def test_sigterm_attempts_cleanup_and_saves_failure(self):
        self.cancel = True
        self.run_mock('cancellation')

    def test_inspect_failure_does_not_gate_removal(self):
        self.failure = 'inspect'
        self.run_mock('inspect unavailable')

    def test_counter_failure_does_not_gate_removal(self):
        self.capture_failure = True
        self.run_mock('counter read unavailable')

    def test_create_timeout_removes_by_persisted_unique_name(self):
        self.failure = 'create'
        record = self.run_mock('create timeout')
        removed = next(command[1] for command, _ in self.calls if command[0] == 'remove-exact')
        self.assertEqual(removed, record['container_name'])

    def test_launcher_wait_failure_still_writes_final_evidence(self):
        self.wait_failure = True
        self.run_mock('owned launcher wait')

    def test_removal_failure_still_writes_final_evidence(self):
        self.remove_failure = True
        self.run_mock('removal cannot be verified')


if __name__ == '__main__':
    unittest.main(verbosity=2)
