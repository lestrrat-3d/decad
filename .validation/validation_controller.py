"""Decad H4 hosted validation; reuse the pinned coil lifecycle, never run locally."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import tarfile
import time
import uuid

import hosted_lifecycle as life

IMAGE = 'golang:1.26.8-bookworm'
MAX_REPORT = 64 << 10


def save_report(path, report):
    data = json.dumps(report, indent=2).encode()
    if len(data) > MAX_REPORT:
        data = b'{"success":false,"error":"report exceeds 64 KiB"}\n'
    path.write_bytes(data)


def require_memory(record):
    if not record.get('final_counters_captured'):
        raise RuntimeError('final cgroup evidence missing')
    if record['fresh_cgroup_peak'] >= life.STOP_AT:
        raise RuntimeError('final memory.peak reached the 3 GiB stop threshold')
    if record['oom_delta'] or record['oom_kill_delta']:
        raise RuntimeError('final cgroup OOM event')


def finish_container(target, directory, cleanup):
    # No inspect/counter operation may precede or gate the exact removal attempt.
    try:
        life.remove_exact_container(target, cleanup)
    except Exception as error:
        cleanup['remove_error'] = str(error)
    query = ['docker', 'ps', '-aq', '--no-trunc', '--filter', 'name=^/' + cleanup['name'] + '$']
    if life.cleanup_checked(query, cleanup):
        raise RuntimeError('exact owned container survived removal')
    cleanup['docker_absent'] = True
    if directory is not None:
        code = ('from pathlib import Path; import sys; p=Path(sys.argv[1]); '
                'print("removed" if not p.exists() else '
                '("empty" if not (p/"cgroup.procs").read_text().strip() else "live"))')
        state = life.cleanup_checked([os.sys.executable, '-c', code, str(directory)], cleanup)
        if state not in ('removed', 'empty'):
            raise RuntimeError('owned cgroup cleanup is unverified')
        cleanup['cgroup_state'] = state
    cleanup['verified'] = True


def run_stage(phase, source, control, scratch, image_id, artifacts, budget, report):
    name = 'decad-h4-' + uuid.uuid4().hex
    record = {'phase': phase['name'], 'container_name': name, 'success': False}
    cleanup = {'name': name}
    report['stages'].append(record)
    # Persist ownership before Docker create: a timed-out create still has one target.
    save_report(artifacts / (phase['name'] + '.json'), record)
    save_report(artifacts / 'report.json', report)
    target = name
    directory = before = None
    primary = None
    started = time.monotonic()
    try:
        if life.CANCELLED:
            raise RuntimeError('hosted job cancellation')
        life.host_headroom()
        create = ['docker', 'create', '--name', name, '--label', 'decad-h4-owner=' + name,
                  '--init', '--user', f'{os.getuid()}:{os.getgid()}',
                  '--memory=4g', '--memory-swap=4g', '--pids-limit=64', '--cpus=2',
                  '--read-only', '--cap-drop=ALL', '--security-opt=no-new-privileges',
                  '--network', 'bridge' if phase['name'] == 'prepare_dependencies' else 'none',
                  '--log-driver=none', '--tmpfs', '/tmp:rw,size=64m',
                  '--mount', f'type=bind,src={source},dst=/src,readonly',
                  '--mount', f'type=bind,src={control},dst=/control,readonly',
                  '--mount', f'type=bind,src={scratch},dst=/out',
                  image_id, 'sleep', 'infinity']
        target_id = life.checked(create, timeout=15)
        if not re.fullmatch('[0-9a-f]{64}', target_id):
            raise RuntimeError('Docker create did not return one ID')
        target = target_id
        record['container_id'] = target
        life.checked(['docker', 'start', target], timeout=15)
        pid = int(life.checked(['docker', 'inspect', '--format', '{{.State.Pid}}', target], timeout=2))
        directory = life.cgroup_path(pid)
        record['limits'] = life.verify_cgroup(directory)
        life.check_members(target, directory)
        before = life.events(directory)
        record['events_before'] = before
        with (artifacts / (phase['name'] + '.jsonl')).open('wb') as stream:
            life.bounded_process(['docker', 'exec', target, 'bash', '/control/stage.sh', phase['name']],
                                 stream, budget, seconds=phase['wall_seconds'], directory=directory,
                                 container=target, record=record, cleanup=cleanup, initial_events=before)
        if life.CANCELLED:
            raise RuntimeError('hosted job cancellation')
    except Exception as error:
        primary = error
        record['error'] = str(error)
    finally:
        try:
            if directory is not None and before is not None and not record.get('final_counters_captured'):
                # The capture helper has its own deadline. Removal gets a fresh reservation.
                try:
                    life.capture_cgroup_bounded(record, directory, before, time.monotonic() + 1.5)
                except Exception as error:
                    record['capture_error'] = str(error)
                    primary = primary or error
        finally:
            try:
                finish_container(target, directory, cleanup)
            except Exception as error:
                record['cleanup_error'] = str(error)
                primary = primary or error
            finally:
                try:
                    require_memory(record)
                    launcher = record.get('launcher', {})
                    if not launcher.get('leader_reaped') or launcher.get('live_after') != []:
                        raise RuntimeError('launcher cleanup unverified')
                    if life.CANCELLED:
                        raise RuntimeError('hosted job cancellation')
                except Exception as error:
                    record['validation_error'] = str(error)
                    primary = primary or error
                finally:
                    record['cleanup'] = {k: v for k, v in cleanup.items() if not k.startswith('_')}
                    record['duration_seconds'] = round(time.monotonic() - started, 3)
                    record['success'] = primary is None and cleanup.get('verified', False)
                    save_report(artifacts / (phase['name'] + '.json'), record)
                    save_report(artifacts / 'report.json', report)
    if primary:
        raise primary


def verify_tests(phase, logfile):
    if not phase.get('tests'):
        return
    passed = set()
    json_lines, wrapper_lines = [], []
    for line in logfile.read_text().splitlines():
        if line.startswith('{'):
            json_lines.append(line)
        else:
            wrapper_lines.append(line)
    # Partition the already capped stream; the parser sees only Go JSON.
    logfile.with_suffix('.go.jsonl').write_text('\n'.join(json_lines) + '\n')
    logfile.with_suffix('.wrapper.log').write_text('\n'.join(wrapper_lines) + '\n')
    for line in json_lines:
        event = json.loads(line)
        if 'Action' not in event:
            raise RuntimeError('non-Go JSON in test event stream')
        if event.get('Action') in ('fail', 'skip'):
            raise RuntimeError('test failed or skipped')
        if event.get('Action') == 'pass' and 'Test' in event:
            passed.add(event['Test'])
    if not set(phase['tests']).issubset(passed):
        raise RuntimeError('expected named test pass missing')


def verify_files(directory, manifest):
    for line in manifest.read_text().splitlines():
        wanted, name = line.split('  ', 1)
        if hashlib.sha256((directory / name).read_bytes()).hexdigest() != wanted:
            raise RuntimeError('source/control hash mismatch: ' + name)



def collect_setup_evidence(scratch, artifacts, report):
    evidence = scratch / 'evidence'
    if not evidence.exists():
        return
    files = sorted(evidence.iterdir())
    if len(files) > 20:
        raise RuntimeError('setup evidence file count exceeded')
    total = 0
    retained = {}
    for path in files:
        limit = 256 << 10 if path.name == 'resolved-modules.json' else 64 << 10
        size = path.stat().st_size
        total += size
        if path.is_symlink() or not path.is_file() or size > limit or total > 512 << 10:
            raise RuntimeError('setup evidence exceeded its file/type/storage limits')
        shutil.copyfile(path, artifacts / path.name)
        retained[path.name] = {'bytes': size, 'sha256': life.sha256(path)}
    report['setup_evidence'] = retained


def main():
    signal.signal(signal.SIGTERM, life.request_cancel)
    signal.signal(signal.SIGINT, life.request_cancel)
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--plan', choices=['section', 'oblique'], required=True)
    parser.add_argument('--manifest-sha256', required=True)
    args = parser.parse_args()
    if os.getenv('GITHUB_ACTIONS') != 'true' or os.getenv('RUNNER_ENVIRONMENT') != 'github-hosted':
        raise RuntimeError('requires an assigned GitHub-hosted slot; never run locally')
    bundle = Path(__file__).resolve().parent
    manifest = bundle / 'SHA256SUMS'
    if life.sha256(manifest) != args.manifest_sha256:
        raise RuntimeError('reviewed control manifest mismatch')
    verify_files(bundle, manifest)
    plan = json.loads((bundle / (args.plan + '-plan.json')).read_text())
    source_manifest = bundle / (args.plan + '-source.sha256')
    source_repo = args.source.resolve()
    if life.checked(['git', 'rev-parse', plan['candidate'] + '^{tree}'], cwd=source_repo) != plan['tree']:
        raise RuntimeError('candidate tree mismatch')
    output = Path(os.environ['RUNNER_TEMP']) / ('decad-h4-' + uuid.uuid4().hex)
    output.mkdir(mode=0o700)
    artifacts = output / 'artifacts'
    artifacts.mkdir()
    report = {'candidate': plan['candidate'], 'base': plan['base'], 'tree': plan['tree'],
              'control_manifest_sha256': args.manifest_sha256, 'image_tag': IMAGE,
              'success': False, 'stages': [], 'artifacts': str(artifacts)}
    budget = life.TextBudget()
    deadline = time.monotonic() + 65 * 60
    scratch_ready = False
    try:
        life.host_headroom()
        scratch = life.prepare_disk(output, report)
        scratch_ready = True
        source = output / 'source'
        source.mkdir()
        # Bound both archive output and the process; extraction reads only this capped archive.
        archive = output / 'source.tar'
        archive_budget = life.TextBudget()
        archive_budget.remaining = 128 << 20
        with archive.open('wb') as stream:
            life.bounded_process(['git', '-C', str(source_repo), 'archive', plan['candidate']],
                                 stream, archive_budget, seconds=30)
        with tarfile.open(archive) as stream:
            stream.extractall(source, filter='data')
        verify_files(source, source_manifest)
        control = output / 'control'
        control.mkdir()
        shutil.copyfile(bundle / 'stage.sh', control / 'stage.sh')
        shutil.copyfile(source_manifest, control / 'source.sha256')
        shutil.copytree(bundle / 'tools', control / 'tools')
        commands = control / 'commands'
        commands.mkdir()
        for phase in plan['phases']:
            cwd = '/out/differential' if phase.get('differential') else '/out/work'
            command = 'cd ' + shlex.quote(cwd) + '\nexec ' + shlex.join(phase['argv']) + '\n'
            (commands / (phase['name'] + '.sh')).write_text('set -euo pipefail\n' + command)
        with (artifacts / 'image-pull.log').open('wb') as stream:
            life.bounded_process(['docker', 'pull', '--quiet', IMAGE], stream, budget, seconds=90)
        image_id = life.checked(['docker', 'image', 'inspect', '--format', '{{.Id}}', IMAGE])
        if not re.fullmatch('sha256:[0-9a-f]{64}', image_id):
            raise RuntimeError('invalid official image ID')
        report['image_id'] = image_id
        report['image_repo_digests'] = json.loads(life.checked(
            ['docker', 'image', 'inspect', '--format', '{{json .RepoDigests}}', image_id]))
        phases = [{'name': 'prepare_tool', 'wall_seconds': 60},
                  {'name': 'prepare_dependencies', 'wall_seconds': 480}] + plan['phases']
        validation_deadline = None
        for phase in phases:
            if time.monotonic() + phase['wall_seconds'] + 60 >= deadline:
                raise RuntimeError('insufficient stage and cleanup time')
            if not phase['name'].startswith('prepare_'):
                if validation_deadline is None:
                    validation_deadline = time.monotonic() + plan['total_wall_seconds']
                if time.monotonic() + phase['wall_seconds'] + 10 >= validation_deadline:
                    raise RuntimeError('insufficient remaining validation-plan time')
            if phase.get('differential'):
                diff = scratch / 'differential'
                shutil.copytree(source, diff)
                for entry in plan['overlays']:
                    origin = (bundle / entry['artifact']).resolve()
                    target = (diff / entry['destination']).resolve()
                    if not origin.is_relative_to(bundle) or not target.is_relative_to(diff) or target.exists():
                        raise RuntimeError('invalid differential overlay path')
                    if life.sha256(origin) != entry['sha256']:
                        raise RuntimeError('differential overlay hash mismatch')
                    target.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copyfile(origin, target)
            run_stage(phase, source, control, scratch, image_id, artifacts, budget, report)
            verify_tests(phase, artifacts / (phase['name'] + '.jsonl'))
            verify_files(source, source_manifest)
            for path in [scratch / 'work', scratch / 'differential']:
                if path.exists():
                    verify_files(path, source_manifest)
        report['success'] = True
    except Exception as error:
        report['error'] = str(error)
        raise
    finally:
        try:
            if scratch_ready:
                collect_setup_evidence(scratch, artifacts, report)
        except Exception as error:
            report['success'] = False
            report['setup_evidence_error'] = str(error)
            raise
        finally:
            finalize_scratch(output, artifacts, report, scratch_ready)


def finalize_scratch(output, artifacts, report, scratch_ready):
    try:
        # This also handles partial loop/mount creation before prepare_disk returned.
        if scratch_ready or (output / 'scratch.ext4').exists() or (output / 'scratch').exists():
            life.remove_work_files(output, report)
    except Exception as error:
        report['success'] = False
        report['scratch_cleanup_error'] = str(error)
        raise
    finally:
        if life.CANCELLED:
            report['success'] = False
            report['cancelled'] = True
        save_report(artifacts / 'report.json', report)

if __name__ == '__main__':
    main()
