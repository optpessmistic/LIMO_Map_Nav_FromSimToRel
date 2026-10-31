"""Collect independent terminal evidence without hiding failed export steps.

The caller owns each operation, its wall deadline, and process cleanup. This
function creates no processes and does not reset or extend those deadlines.
Interrupts propagate to the caller's shutdown path.
"""
from math import isfinite
import time


def collect_terminal_evidence(export_steps, wait_for_tail, *, deadline_wall, monotonic=time.monotonic):
    """Attempt every bounded export, then the independent stationary tail.

    An export error must not skip the tail. A later successful step never
    converts an earlier failed step into success. The tail callback must return
    exactly True only after verifying its original terminal/stationarity rules.
    """
    if type(deadline_wall) not in (int, float) or not isfinite(deadline_wall):
        raise ValueError('finite absolute terminal deadline required')
    report = {'exports': [], 'tail_verified': False, 'errors': [], 'deadline_wall': deadline_wall}
    for name, operation in export_steps:
        entry = {'name': name, 'passed': False}
        try:
            if monotonic() >= deadline_wall:
                raise TimeoutError('terminal deadline before export launch')
            operation(deadline_wall)
            if monotonic() > deadline_wall:
                raise TimeoutError('export completed after terminal deadline')
            entry['passed'] = True
        except Exception as exc:
            entry['error'] = type(exc).__name__ + ': ' + str(exc)
            report['errors'].append(name + ': ' + entry['error'])
        report['exports'].append(entry)
    try:
        if monotonic() >= deadline_wall:
            raise TimeoutError('terminal deadline before tail verification')
        report['tail_verified'] = wait_for_tail(deadline_wall) is True and monotonic() <= deadline_wall
        if not report['tail_verified']:
            report['errors'].append('terminal_stationary_tail_not_verified')
    except Exception as exc:
        report['errors'].append('terminal_tail: ' + type(exc).__name__ + ': ' + str(exc))
    report['passed'] = not report['errors']
    return report
