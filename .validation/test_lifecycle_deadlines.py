"""Mock-only controller checks: no containers, builds, network, or signals."""
from pathlib import Path
import io
import tempfile
import types
import unittest
from unittest.mock import patch, MagicMock

ROOT = Path(__file__).parent
import hosted_lifecycle as controller


class Deadlines(unittest.TestCase):
    def setUp(self):
        controller.CANCELLED = True
        controller.CANCEL_DEADLINE = 99.0
        self.clock = patch.object(controller.time, 'monotonic', return_value=100.0)
        self.clock.start()
        self.addCleanup(self.clock.stop)

    def fake_process(self):
        proc = MagicMock()
        proc.pid = 987654
        proc.stdout, proc.stderr = io.BytesIO(), io.BytesIO()
        proc.wait.return_value = 0
        return proc

    def test_reserved_group_cleanup_ignores_expired_cancellation(self):
        proc = self.fake_process()
        with patch.object(controller.os, 'killpg') as signals, patch.object(controller, 'live_group_members', return_value=[]) as members:
            controller.stop_owned_group(proc, hard_deadline=100.4, ignore_cancel_deadline=True)
        self.assertEqual(members.call_args.args[1], 100.4)
        self.assertGreater(proc.wait.call_args.kwargs['timeout'], 0)
        signals.assert_called_once()

    def test_normal_group_cleanup_still_obeys_cancellation(self):
        proc = self.fake_process()
        with patch.object(controller.os, 'killpg'), patch.object(controller, 'live_group_members', return_value=[]) as members:
            controller.stop_owned_group(proc, hard_deadline=102)
        self.assertEqual(members.call_args.args[1], 99)

    def check_command(self, allow_cancel):
        proc = self.fake_process()
        selector = MagicMock()
        selector.get_map.return_value = {}
        with patch.object(controller.subprocess, 'Popen', return_value=proc), patch.object(controller.selectors, 'DefaultSelector', return_value=selector), patch.object(controller, 'leader_status', return_value=object()), patch.object(controller, 'stop_owned_group', return_value=0) as stop:
            if allow_cancel:
                self.assertEqual(controller.checked(['mock-control'], timeout=3, allow_cancel=True, hard_deadline=102), '')
            else:
                with self.assertRaisesRegex(RuntimeError, 'hosted job cancellation'):
                    controller.checked(['mock-control'], timeout=3, hard_deadline=102)
        self.assertEqual(stop.call_args.kwargs['hard_deadline'], 102)
        self.assertEqual(stop.call_args.kwargs['ignore_cancel_deadline'], allow_cancel)

    def test_checked_propagates_reserved_cleanup(self):
        self.check_command(True)

    def test_checked_preserves_normal_cancellation(self):
        self.check_command(False)

    def test_pause_failure_still_removes_and_reaps_with_reserved_time(self):
        proc = self.fake_process()
        selector = MagicMock()
        selector.get_map.return_value = {}
        calls = []
        def checked(command, **kwargs):
            calls.append((command, kwargs))
            if command[:2] == ['docker', 'pause']:
                raise RuntimeError('simulated pause timeout')
            return ''
        cleanup, record = {}, {}
        with patch.object(controller.subprocess, 'Popen', return_value=proc), patch.object(controller.selectors, 'DefaultSelector', return_value=selector), patch.object(controller, 'leader_status', return_value=object()), patch.object(controller, 'status_code', return_value=1), patch.object(controller, 'checked', side_effect=checked), patch.object(controller, 'stop_owned_group', return_value=1) as stop:
            with self.assertRaisesRegex(RuntimeError, 'unexpected launcher exit'):
                controller.bounded_process(['mock-work'], io.BytesIO(), controller.TextBudget(), seconds=2, container='owned-id', cleanup=cleanup, record=record)
        self.assertEqual(calls[1][0], ['docker', 'rm', '-f', 'owned-id'])
        self.assertEqual(calls[1][1]['timeout'], 2)
        self.assertTrue(calls[1][1]['allow_cancel'])
        self.assertEqual(calls[1][1]['hard_deadline'], 103.5)
        self.assertEqual(controller.cleanup_remaining(cleanup), 3.5)
        self.assertTrue(stop.call_args.kwargs['ignore_cancel_deadline'])
        self.assertEqual(stop.call_args.kwargs['hard_deadline'], 103.5)
        self.assertIn('simulated pause timeout', record['cgroup_metrics_error'])

    def test_filesystem_cleanup_commands_ignore_expired_cancellation(self):
        with tempfile.TemporaryDirectory(dir=ROOT, prefix='mock-owned-') as directory:
            output = Path(directory)
            image = output / 'scratch.ext4'
            image.write_bytes(b'')
            (output / 'scratch').mkdir()
            calls, queries = [], []
            def checked(command, **kwargs):
                calls.append((command, kwargs))
                if command[:3] == ['sudo', 'losetup', '-j']:
                    queries.append(command)
                    return f'/dev/loop42: [] ({image})' if len(queries) == 1 else ''
                return ''
            report = {}
            with patch.object(controller.os.path, 'ismount', side_effect=[True, False]), patch.object(controller, 'disk_usage', return_value={}), patch.object(controller, 'checked', side_effect=checked):
                controller.remove_work_files(output, report)
            self.assertTrue(report['scratch_cleanup']['verified'])
            self.assertTrue(all(options['allow_cancel'] for _, options in calls))
            self.assertIn(['sudo', 'losetup', '--detach', '/dev/loop42'], [command for command, _ in calls])
            self.assertFalse(image.exists())


if __name__ == '__main__':
    unittest.main(verbosity=2)
