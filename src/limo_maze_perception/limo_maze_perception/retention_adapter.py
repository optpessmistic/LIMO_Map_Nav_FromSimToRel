"""Single executor owner + one bounded, non-daemon owned subprocess writer.

Core is never touched by the writer. Artifact/checkpoint files are written there.
Transport is bounded nonblocking pipes, no feeder or I/O threads. This adapter
does not prove artifacts, DDS delivery, camera coverage, or mission success.
"""
import base64
from collections import deque
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import uuid

from limo_evidence_protocol.retention import Core, Policy, canonical, sha, validate_request

MAX_CHECKPOINT = 4 * 1024 * 1024
MAX_REPLY = 32768


@dataclass(frozen=True)
class RuntimePolicy:
    worker_timeout_s: float = 5.0
    term_grace_s: float = .5
    kill_grace_s: float = .5
    shutdown_timeout_s: float = 8.0

    def __post_init__(self):
        for value in (self.worker_timeout_s, self.term_grace_s, self.kill_grace_s, self.shutdown_timeout_s):
            if type(value) not in (int, float) or not math.isfinite(value) or not 0 < value <= 60:
                raise ValueError('runtime_durations_must_be_in_0_60')


class OwnedWriter:
    """One child, no shell, no group signalling; pidfd pins exactly that child."""
    def __init__(self, root, snapshot, job, policy, now, *, worker_path=None):
        if not hasattr(os, 'pidfd_open') or not hasattr(signal, 'pidfd_send_signal'):
            raise RuntimeError('Linux_pidfd_required')
        self.job, self.policy, self.started = job, policy, now
        self.token, self.operation_id = uuid.uuid4().hex, uuid.uuid4().hex
        meta = None if job is None else dict(request_id=job.request_id,
            request_sha256=job.request_sha256, directory=job.directory,
            files=[(name, len(raw)) for name, raw in job.files],
            manifest_bytes=len(job.manifest), receipt_bytes=len(job.receipt), file_bytes=job.file_bytes)
        header = canonical(dict(operation_id=self.operation_id, token=self.token,
                                checkpoint_bytes=len(snapshot), job=meta), 16384)
        segments = (len(header).to_bytes(8, 'big'), header, snapshot)
        if job is not None:
            segments += tuple(raw for _, raw in job.files) + (job.manifest, job.receipt)
        self.segments = deque(memoryview(raw) for raw in segments)
        self.output, self.errors = bytearray(), bytearray()
        self.fault = self.stop_reason = None
        self.stop_started = self.killed_at = None
        self.unconfirmed = False
        self.reply = None
        self.returncode = None
        self.pidfd = None
        self.setup_error = None
        worker_path = worker_path or Path(__file__).with_name('retention_worker.py')
        env = os.environ.copy()
        env['LIMO_RETENTION_WRITER_TOKEN'] = self.token
        self.process = subprocess.Popen([sys.executable, '-B', str(worker_path), '--root', str(root),
            '--max-wire-bytes', str(sum(len(raw) for raw in segments))],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            start_new_session=True, env=env)
        # An unreaped owned child cannot have its PID reused before pidfd_open.
        try:
            self.pidfd = os.pidfd_open(self.process.pid)
            for stream in (self.process.stdin, self.process.stdout, self.process.stderr):
                os.set_blocking(stream.fileno(), False)
        except OSError as exc:
            # Preserve the actual child object even on setup failure. With no
            # pidfd we refuse signals, close our pipes, and wait for exact exit.
            self.setup_error = self.fault = 'writer_setup_failed:' + type(exc).__name__
            for stream in (self.process.stdin, self.process.stdout, self.process.stderr):
                stream.close()

    def signal(self, sig):
        if self.process.poll() is None:
            if self.pidfd is None:
                self.unconfirmed = True
                return
            try:
                signal.pidfd_send_signal(self.pidfd, sig)
            except ProcessLookupError:
                pass  # Exact child exited between poll and pidfd signalling.
            except PermissionError:
                self.fault = self.fault or 'owned_writer_signal_permission_denied'
                self.unconfirmed = True

    def stop(self, reason, now):
        if self.stop_reason is None:
            self.stop_reason, self.stop_started = reason, now
            self.signal(signal.SIGTERM)

    def _drain(self, stream, target, limit):
        for _ in range(8):
            try:
                raw = os.read(stream.fileno(), 8192)
            except BlockingIOError:
                break
            if not raw:
                break
            if len(target) + len(raw) > limit:
                self.fault = self.fault or 'writer_reply_budget'
            target.extend(raw[:max(0, limit - len(target))])

    def poll(self, now):
        if self.setup_error:
            self.returncode = self.process.poll()
            if self.returncode is not None:
                return True
            self.stop(self.setup_error, now)
            return False
        # At most 1 MiB transfer per owner tick. No disk or blocking IPC here.
        remaining = 1024 * 1024
        while self.segments and remaining and self.stop_reason is None:
            view = self.segments[0]
            try:
                sent = os.write(self.process.stdin.fileno(), view[:min(65536, remaining)])
            except BlockingIOError:
                break
            except (BrokenPipeError, OSError) as exc:
                self.fault = self.fault or 'writer_input_closed:' + type(exc).__name__
                break
            remaining -= sent
            if sent == len(view):
                self.segments.popleft()
            else:
                self.segments[0] = view[sent:]
        if not self.segments and not self.process.stdin.closed:
            self.process.stdin.close()
        self._drain(self.process.stdout, self.output, MAX_REPLY)
        self._drain(self.process.stderr, self.errors, 4096)
        self.returncode = self.process.poll()
        if self.returncode is not None:
            # The child can write its last reply between the first drain and
            # exit poll. With no child descendants, all output is now buffered.
            self._drain(self.process.stdout, self.output, MAX_REPLY)
            self._drain(self.process.stderr, self.errors, 4096)
            try:
                self.reply = json.loads(self.output)
                if type(self.reply) is not dict or type(self.reply.get('ok')) is not bool:
                    raise ValueError('completion_object_required')
                if self.reply.get('ok') and self.reply.get('operation_id') != self.operation_id:
                    self.fault = self.fault or 'writer_reply_identity'
            except (ValueError, UnicodeError, RecursionError):
                self.reply = None
                self.fault = self.fault or 'writer_reply_missing_or_invalid'
            return True
        if self.fault or now - self.started >= self.policy.worker_timeout_s:
            self.stop(self.fault or 'writer_wall_timeout', now)
        if self.stop_started is not None and now - self.stop_started >= self.policy.term_grace_s:
            if self.killed_at is None:
                self.signal(signal.SIGKILL)
                self.killed_at = now
            elif now - self.killed_at >= self.policy.kill_grace_s:
                self.unconfirmed = True
        return False

    def snapshot(self):
        return dict(pid=self.process.pid, operation_id=self.operation_id, owner_token=self.token,
                    started_wall=self.started,
                    job_id=self.job.request_id if self.job else None, returncode=self.returncode,
                    stop_reason=self.stop_reason, forced_kill=self.killed_at is not None,
                    termination_unconfirmed=self.unconfirmed, fault=self.fault,
                    stderr=self.errors.decode(errors='replace'))

    def dispose(self):
        if self.process.poll() is None:
            raise RuntimeError('cannot_release_live_writer')
        for stream in (self.process.stdin, self.process.stdout, self.process.stderr):
            stream.close()
        if self.pidfd is not None:
            os.close(self.pidfd)
        self.pidfd = None


class ProducerRetention:
    def __init__(self, output, session_id, policy=Policy(), runtime=RuntimePolicy(), *, worker_path=None):
        self.owner_thread = threading.get_ident()
        self.session_id, self.policy, self.runtime = session_id, policy, runtime
        output = Path(output).expanduser().absolute()
        if output.is_symlink() or output.resolve() != output:
            raise ValueError('canonical_output_path_required')
        output.mkdir(parents=True, exist_ok=False)
        self.output = output
        self.worker_path = worker_path
        self.core = self.worker = None
        self.closed = False
        self.reason = self.fault = None
        self.revision, self.persisted_revision = 0, -1
        self.worker_revision = None
        self.worker_results = None
        self.worker_fault = None
        self.durable_results = {}
        self.durable_fault = None
        self.last_core_fingerprint = None
        self.writer_history = deque(maxlen=16)
        self.protocol_rejections = 0
        self.last_protocol_error = None
        self.checkpoint_failed = False

    def assert_owner(self):
        if threading.get_ident() != self.owner_thread:
            raise RuntimeError('Core_owner_must_be_serial_executor_thread')

    def dirty(self):
        self.revision += 1

    def on_epoch(self, epoch, ready, now):
        self.assert_owner()
        if self.closed:
            return
        if self.core is None:
            if ready:
                self.core = Core(epoch, self.session_id, self.policy)
                self.dirty()
        elif not ready or epoch != self.core.epoch:
            self.close('epoch_invalid_or_changed', now)

    def offer(self, record, rgb, depth, now):
        self.assert_owner()
        if self.closed or self.core is None:
            return {'available': False, 'reason': self.reason or 'epoch_not_bound'}
        if record.get('status') != 'processed':
            return {'available': False, 'reason': 'not_processed_record'}
        try:
            result = self.core.offer(record, rgb, depth, now)
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            self.close('source_offer_invalid:' + str(exc)[:128], now)
            return {'available': False, 'reason': self.reason}
        return result

    def request(self, payload, now):
        self.assert_owner()
        try:
            if type(payload) is str:
                if len(payload) > 16384:
                    raise ValueError('request_transport_budget')
                payload = json.loads(payload)
            request, raw = validate_request(payload)
        except (ValueError, TypeError, KeyError, RecursionError) as exc:
            self.protocol_rejections = min(self.protocol_rejections + 1, 2**63 - 1)
            self.last_protocol_error = type(exc).__name__ + ':' + str(exc)[:128]
            self.dirty()
            return {'status': 'rejected', 'reason': self.last_protocol_error, 'recorded': False}
        if self.core is None:
            # No valid producer epoch ever authorized this ID. Do not later
            # reconstruct a Core and silently turn this rejection into success.
            self.close('request_before_epoch_binding', now)
            return dict(retention_id=request['retention_id'], request_sha256=sha(raw),
                        status='failed', reason=self.reason, recorded=False, admission_closed=True)
        result = self.core.request(request, now)
        self.dirty()
        return self.transport_result(result)

    def transport_result(self, result):
        value = dict(result)
        durable = (self.durable_results.get(value.get('retention_id')) == sha(canonical(result)) and
                   self.durable_fault == self.fault)
        value['checkpoint_persisted'] = durable
        if value['status'] == 'stored' and not durable:
            value['status'] = 'checkpoint_pending'
        if self.fault:
            value['status'], value['reason'] = 'failed', self.fault
        return value

    def snapshot(self):
        return dict(schema_version=1, session_id=self.session_id, closed=self.closed,
                    reason=self.reason, fault=self.fault, revision=self.revision,
                    persisted_revision=self.persisted_revision,
                    checkpoint_failed=self.checkpoint_failed,
                    protocol_rejections=self.protocol_rejections, last_protocol_error=self.last_protocol_error,
                    writer=self.worker.snapshot() if self.worker else None,
                    writer_history=list(self.writer_history), core=self.core.checkpoint() if self.core else None,
                    scope='source_bytes_retention_only; no_artifact_audit_or_mission_success_claim')

    def close(self, reason, now):
        self.assert_owner()
        previous_fault = self.fault
        if reason not in ('shutdown', 'SIGINT', 'SIGTERM'):
            self.fault = self.fault or reason
        if self.fault != previous_fault:
            self.dirty()
        if not self.closed:
            self.closed, self.reason = True, reason
            if self.core:
                self.core.close(now)
            self.dirty()
        if self.worker and self.worker.job is not None:
            self.worker.stop(reason, now)

    def tick(self, now):
        self.assert_owner()
        if self.core:
            self.core.expire(now)
        if self.worker:
            worker = self.worker
            if not worker.poll(now):
                if worker.stop_reason and not self.closed:
                    self.fault = worker.stop_reason
                    self.close(self.fault, now)
                return
            reply = worker.reply or {}
            success = bool(worker.returncode == 0 and reply.get('ok') and
                           not worker.fault and worker.stop_reason is None)
            if success:
                self.persisted_revision = self.worker_revision
                self.durable_results.update(self.worker_results)
                self.durable_fault = self.worker_fault
            else:
                self.fault = self.fault or worker.fault or worker.stop_reason or reply.get('error', 'writer_failed')
                # A failed checkpoint is not retried indefinitely. Keep the last
                # durable snapshot, explicit failure, and refuse further success.
                if worker.job is None:
                    self.checkpoint_failed = True
            if worker.job is not None:
                receipt = None
                if success:
                    try:
                        receipt = base64.b64decode(reply['receipt_b64'], validate=True)
                    except (ValueError, KeyError, TypeError):
                        self.fault = self.fault or 'invalid_writer_receipt'
                # Exit confirmed before releasing Core's writing ownership.
                self.core.finish(worker.job, receipt, now)
            if worker.job is not None or not success:
                self.writer_history.append(worker.snapshot())
            worker.dispose()
            self.worker = None
            if worker.job is not None or not success:
                self.dirty()
            if self.fault:
                self.close(self.fault, now)
        if self.core:
            # Candidate TTL is transient. Persist every request/ownership/fault
            # transition, not every cache offer or source frame.
            state = self.core.checkpoint()
            fingerprint = sha(canonical({key: state[key] for key in
                ('closed', 'writer_pending', 'jobs_reserved', 'file_bytes_reserved', 'admission', 'results')}, MAX_CHECKPOINT))
            if fingerprint != self.last_core_fingerprint:
                self.last_core_fingerprint = fingerprint
                self.dirty()
        if self.checkpoint_failed:
            return
        job = self.core.take(now) if self.core and not self.closed else None
        if job is not None:
            self.dirty()
        if job is None and self.revision == self.persisted_revision:
            return
        raw = canonical(self.snapshot(), MAX_CHECKPOINT)
        self.worker_revision = self.revision
        self.worker_fault = self.fault
        self.worker_results = ({r['retention_id']: sha(canonical(r)) for r in self.core.checkpoint()['results']}
                               if self.core else {})
        try:
            self.worker = OwnedWriter(self.output, raw, job, self.runtime, now, worker_path=self.worker_path)
        except (OSError, RuntimeError) as exc:
            # Constructor retains a child on all post-spawn setup failures;
            # reaching here means no owned process was launched.
            if job is not None:
                self.core.finish(job, None, now)
            self.fault = 'writer_start_failed:' + type(exc).__name__
            self.checkpoint_failed = True
            self.close(self.fault, now)

    def results(self):
        self.assert_owner()
        return [self.transport_result(r) for r in self.core.checkpoint()['results']] if self.core else []

    def shutdown(self, reason='shutdown'):
        self.assert_owner()
        started = time.monotonic()
        self.close(reason, started)
        deadline = started + self.runtime.shutdown_timeout_s
        while time.monotonic() < deadline:
            self.tick(time.monotonic())
            if self.worker is None and (self.checkpoint_failed or self.revision == self.persisted_revision):
                break
            time.sleep(.01)
        if self.worker:
            self.worker.stop('shutdown_wall_timeout', time.monotonic())
            # Do not release/reap an unconfirmed live writer or spawn a second.
            self.worker.signal(signal.SIGKILL)
        report = self.snapshot()
        report['shutdown'] = dict(elapsed_wall_s=time.monotonic() - started,
            owned_writer_exit_confirmed=self.worker is None,
            final_checkpoint_persisted=self.worker is None and self.revision == self.persisted_revision,
            limitation='SIGKILL cannot guarantee bounded exit from uninterruptible kernel I/O')
        return report
