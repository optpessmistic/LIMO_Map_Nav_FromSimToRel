"""Run the two failing existing test methods, preserving harness stderr."""
from pathlib import Path
import json
import sys
import unittest

root = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(root/'tests'))
import test_frame_retention_adapter as tests

original = tests.cleanup_signal_harness
def capture_cleanup(process, identity_path, worker_path):
    result = original(process, identity_path, worker_path)
    print('HARNESS_DIAGNOSTIC '+json.dumps(dict(returncode=process.returncode,
        stdout=result[0].decode(errors='replace'), stderr=result[1].decode(errors='replace'))), flush=True)
    return result
tests.cleanup_signal_harness = capture_cleanup
suite = (unittest.defaultTestLoader.loadTestsFromNames([
    'test_retention_pipe', 'test_frame_retention_adapter', 'test_frame_retention_owner_races'])
    if '--all' in sys.argv else unittest.TestSuite(tests.AdapterTests(name) for name in (
    'test_missing_ready_assertion_still_flushes_worker_and_closes_pipes',
    'test_real_signals_use_extracted_main_and_leave_no_owned_writer')))
result = unittest.TextTestRunner(verbosity=2).run(suite)
raise SystemExit(0 if result.wasSuccessful() else 1)
