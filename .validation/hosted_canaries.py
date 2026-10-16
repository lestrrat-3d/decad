"""Small live cleanup canaries, hosted only, pending review and slot assignment."""
import argparse
import json
import os
from pathlib import Path
import signal
import threading
import time
import uuid

import validation_controller as ctrl


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--manifest-sha256', required=True)
    args = parser.parse_args()
    life = ctrl.life
    if os.getenv('GITHUB_ACTIONS') != 'true' or os.getenv('RUNNER_ENVIRONMENT') != 'github-hosted':
        raise RuntimeError('live canaries require an assigned hosted slot')
    signal.signal(signal.SIGTERM, life.request_cancel)
    signal.signal(signal.SIGINT, life.request_cancel)
    bundle = Path(__file__).resolve().parent
    if life.sha256(bundle / 'SHA256SUMS') != args.manifest_sha256:
        raise RuntimeError('control manifest differs from reviewed canary')
    ctrl.verify_files(bundle, bundle / 'SHA256SUMS')
    output = Path(os.environ['RUNNER_TEMP']) / ('decad-h4-canary-' + uuid.uuid4().hex)
    output.mkdir(mode=0o700)
    artifacts = output / 'artifacts'
    artifacts.mkdir()
    report = {'success': False, 'stages': [], 'artifacts': str(artifacts),
              'control_manifest_sha256': args.manifest_sha256}
    watcher = None
    stop_watcher = threading.Event()
    try:
        life.host_headroom()
        scratch = life.prepare_disk(output, report)
        source = output / 'source'
        source.mkdir()
        control = output / 'control'
        control.mkdir()
        (control / 'stage.sh').write_bytes((bundle / 'canary_stage.sh').read_bytes())
        budget = life.TextBudget()
        with (artifacts / 'image-pull.log').open('wb') as stream:
            life.bounded_process(['docker', 'pull', '--quiet', ctrl.IMAGE], stream, budget, seconds=90)
        image_id = life.checked(['docker', 'image', 'inspect', '--format', '{{.Id}}', ctrl.IMAGE])
        report['image_id'] = image_id
        for case in ['failure', 'sigterm']:
            stop_watcher.clear()
            life.CANCELLED = False
            life.CANCEL_DEADLINE = None
            if case == 'sigterm':
                def cancel_owned_controller():
                    deadline = time.monotonic() + 45
                    while not stop_watcher.wait(0.05):
                        if (scratch / 'canary.started').exists():
                            os.kill(os.getpid(), signal.SIGTERM)
                            return
                        if time.monotonic() >= deadline:
                            return
                watcher = threading.Thread(target=cancel_owned_controller, daemon=True)
                watcher.start()
            caught = None
            try:
                ctrl.run_stage({'name': case, 'wall_seconds': 10}, source, control, scratch,
                               image_id, artifacts, budget, report)
            except Exception as error:
                caught = error
            finally:
                stop_watcher.set()
                if watcher:
                    watcher.join(timeout=1)
                    if watcher.is_alive():
                        raise RuntimeError('canary signal watcher did not stop')
            record = report['stages'][-1]
            expected = 'unexpected launcher exit 23' if case == 'failure' else 'cancellation'
            if caught is None or expected not in str(caught):
                raise RuntimeError(f'{case}: expected failure not observed: {caught}')
            if record['success'] or not record['cleanup'].get('verified'):
                raise RuntimeError(f'{case}: failed workload cleanup unverified')
            ctrl.require_memory(record)
            if 'capture_error' in record or 'cgroup_metrics_error' in record:
                raise RuntimeError(f'{case}: final counter capture failed')
            record['expected_canary_failure_verified'] = True
        report['success'] = True
    except Exception as error:
        report['error'] = str(error)
        raise
    finally:
        stop_watcher.set()
        try:
            life.remove_work_files(output, report)
        except Exception as error:
            report['success'] = False
            report['scratch_cleanup_error'] = str(error)
            raise
        finally:
            ctrl.save_report(artifacts / 'canary-report.json', report)


if __name__ == '__main__':
    main()
