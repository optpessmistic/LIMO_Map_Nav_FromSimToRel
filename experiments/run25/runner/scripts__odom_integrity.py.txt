"""ROS-independent integrity check for a finite, received-order odometry sample.

Integer nanoseconds preserve distinct stamps even at large simulation times.
Repeated stamps must have exactly the same pose/twist components; there is no
epsilon that could hide conflicting producers. Identical repeats are counted,
but never increase the unique-sample count or the observed time span. Failures
are sticky for this instance. This checks the received prefix, not unseen DDS
messages, publisher identity, physical accuracy, stationarity or a whole run.
"""

from collections import Counter
import math


MIN_UNIQUE_ODOM = 50
MIN_OBSERVATION_NS = 1_000_000_000
POSE_FIELDS = ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')
TWIST_FIELDS = ('vx', 'vy', 'vz', 'wx', 'wy', 'wz')


def stamp_to_nanoseconds(sec, nanosec):
    """Validate ROS-style simulation time without converting through float."""
    if (type(sec) is not int or type(nanosec) is not int or
            sec < 0 or not 0 <= nanosec < 1_000_000_000):
        raise ValueError('stamp needs nonnegative integer sec and nanosec in [0, 1e9)')
    return sec * 1_000_000_000 + nanosec


def _vector(values, size):
    try:
        result = tuple(values)
        if len(result) != size or any(type(v) not in (int, float) for v in result):
            raise ValueError('wrong number or type of components')
        return tuple(float(v) for v in result)
    except (TypeError, OverflowError) as exc:
        raise ValueError('invalid numeric vector') from exc


def _json_number(value):
    return value if math.isfinite(value) else None


class OdomIntegrity:
    """Accumulate received odometry using immutable first values at each stamp.

    ``observe`` accepts pose=(x,y,z,qx,qy,qz,qw) and
    twist=(vx,vy,vz,wx,wy,wz). It does not normalize quaternions or replace a
    conflicting first value with a later one. Frame validation and publisher
    cardinality remain the caller's responsibility.
    """

    def __init__(self):
        self.counts = Counter()
        self._first = {}
        self._valid_stamps = set()
        self._repeated_stamps = set()
        self._conflicting_stamps = set()
        self._watermark_ns = None
        self._first_received_ns = None
        self._last_received_ns = None
        self._examples = []
        self._maxima = {key: 0. for key in (
            'position_delta_m', 'quaternion_component_delta',
            'linear_twist_delta_m_s', 'angular_twist_delta_rad_s')}

    def _example(self, kind, stamp_ns, **details):
        if len(self._examples) < 8:
            self._examples.append({'kind': kind, 'stamp_ns': stamp_ns, **details})

    def observe(self, stamp_ns, pose, twist):
        """Record one arrival; return whether it is a valid, new, advancing stamp.

        Invalid data are counted instead of raising, allowing callers to produce
        a complete failure report. Passing None records an invalid ROS stamp.
        """
        self.counts['messages'] += 1
        valid_stamp = type(stamp_ns) is int and stamp_ns >= 0
        if not valid_stamp:
            self.counts['invalid_stamp_messages'] += 1
            self._example('invalid_stamp', None)
        payload = None
        try:
            pose, twist = _vector(pose, 7), _vector(twist, 6)
            nonfinite = [name for name, value in zip(POSE_FIELDS+TWIST_FIELDS, pose+twist)
                         if not math.isfinite(value)]
            if nonfinite:
                self.counts['nonfinite_messages'] += 1
                self.counts['nonfinite_components'] += len(nonfinite)
                self._example('nonfinite', stamp_ns if valid_stamp else None, fields=nonfinite)
            else:
                payload = (pose, twist)
        except ValueError:
            self.counts['invalid_payload_messages'] += 1
            self._example('invalid_payload', stamp_ns if valid_stamp else None)
        if not valid_stamp:
            return False

        self._last_received_ns = stamp_ns
        if self._first_received_ns is None:
            self._first_received_ns = stamp_ns
        backward = self._watermark_ns is not None and stamp_ns < self._watermark_ns
        if backward:
            self.counts['backward_messages'] += 1
            self._example('backward_stamp', stamp_ns, watermark_ns=self._watermark_ns)
        self._watermark_ns = max(stamp_ns, self._watermark_ns or 0)
        repeated = stamp_ns in self._first
        if repeated:
            self.counts['duplicate_messages'] += 1
            self._repeated_stamps.add(stamp_ns)
            baseline = self._first[stamp_ns]
            # Nonfinite/malformed payloads already fail; do not manufacture a
            # numeric delta or quietly select a later finite value as baseline.
            if baseline is not None and payload is not None:
                old_pose, old_twist = baseline
                pose_changed, twist_changed = pose != old_pose, twist != old_twist
                if pose_changed:
                    self.counts['pose_conflict_messages'] += 1
                if twist_changed:
                    self.counts['twist_conflict_messages'] += 1
                if pose_changed or twist_changed:
                    self.counts['conflicting_duplicate_messages'] += 1
                    self._conflicting_stamps.add(stamp_ns)
                    deltas = {
                        'position_delta_m': math.dist(pose[:3], old_pose[:3]),
                        'quaternion_component_delta': max(abs(a-b) for a, b in zip(pose[3:], old_pose[3:])),
                        'linear_twist_delta_m_s': math.dist(twist[:3], old_twist[:3]),
                        'angular_twist_delta_rad_s': math.dist(twist[3:], old_twist[3:]),
                    }
                    for name, value in deltas.items():
                        self._maxima[name] = max(self._maxima[name], value)
                    fields = [name for name, a, b in zip(POSE_FIELDS+TWIST_FIELDS, pose+twist, old_pose+old_twist) if a != b]
                    self._example('conflicting_duplicate', stamp_ns, changed_fields=fields,
                                  deltas={k: _json_number(v) for k, v in deltas.items()},
                                  delta_overflow=any(not math.isfinite(v) for v in deltas.values()))
                else:
                    self.counts['identical_duplicate_messages'] += 1
        else:
            self._first[stamp_ns] = payload
        if payload is not None:
            self._valid_stamps.add(stamp_ns)
        return payload is not None and not repeated and not backward

    @property
    def span_ns(self):
        return max(self._valid_stamps)-min(self._valid_stamps) if self._valid_stamps else 0

    @property
    def observation_sufficient(self):
        """Sample size only; callers must still inspect errors before passing."""
        return len(self._valid_stamps) >= MIN_UNIQUE_ODOM and self.span_ns >= MIN_OBSERVATION_NS

    def errors(self):
        result = []
        if len(self._valid_stamps) < MIN_UNIQUE_ODOM:
            result.append(f'odom: expected >={MIN_UNIQUE_ODOM} valid unique integer stamps, received {len(self._valid_stamps)}')
        if self.span_ns < MIN_OBSERVATION_NS:
            result.append(f'odom: observed valid stamp span {self.span_ns} ns is shorter than {MIN_OBSERVATION_NS} ns')
        for key, description in (
                ('invalid_stamp_messages', 'invalid integer timestamp'),
                ('invalid_payload_messages', 'malformed pose/twist'),
                ('nonfinite_messages', 'nonfinite pose/twist'),
                ('backward_messages', 'timestamp below received high-water mark'),
                ('pose_conflict_messages', 'different pose at the same integer timestamp'),
                ('twist_conflict_messages', 'different twist at the same integer timestamp')):
            if self.counts[key]:
                result.append(f'odom: {self.counts[key]} messages with {description}')
        return result

    def report(self):
        """JSON-safe diagnostics for this sampled prefix, including bad inputs."""
        counters = {key: self.counts[key] for key in (
            'messages', 'duplicate_messages', 'identical_duplicate_messages',
            'conflicting_duplicate_messages', 'pose_conflict_messages', 'twist_conflict_messages',
            'backward_messages', 'nonfinite_messages', 'nonfinite_components',
            'invalid_stamp_messages', 'invalid_payload_messages')}
        return {
            'passed': not self.errors(), 'errors': self.errors(),
            'counts': counters, 'unique_stamps': len(self._first),
            'valid_unique_stamps': len(self._valid_stamps),
            'repeated_stamps': len(self._repeated_stamps),
            'conflicting_stamps': len(self._conflicting_stamps),
            'first_received_stamp_ns': self._first_received_ns,
            'last_received_stamp_ns': self._last_received_ns,
            'minimum_valid_stamp_ns': min(self._valid_stamps) if self._valid_stamps else None,
            'maximum_valid_stamp_ns': max(self._valid_stamps) if self._valid_stamps else None,
            'valid_span_ns': self.span_ns, 'valid_span_seconds': self.span_ns/1_000_000_000,
            'minimum_unique_stamps': MIN_UNIQUE_ODOM, 'minimum_span_ns': MIN_OBSERVATION_NS,
            'observation_sufficient': self.observation_sufficient,
            'same_stamp_comparison': 'exact numeric equality of all 7 pose and 6 twist components; first value retained',
            'maximum_same_stamp_deltas': {k: _json_number(v) for k, v in self._maxima.items()},
            'delta_overflow': any(not math.isfinite(v) for v in self._maxima.values()),
            'diagnostic_examples': list(self._examples),
            'limits': 'Received prefix only; identical duplicates do not advance observation. No whole-run, lossless-delivery, physical-motion or publisher-provenance claim.',
        }
