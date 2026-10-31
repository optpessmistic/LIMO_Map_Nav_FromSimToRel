# Exact frame-retention audit review

Reviewed `tools/audit_frame_retention.py` SHA-256 `0fbb3c07ac7f40726d0ba2fda93b89f7582ab570db07c091608fab6f5d15a232`. The independent offline run verified the source was unchanged during the checks. No ROS/Gazebo was initialized, no production source was edited, and all image artifacts were synthetic temporary fixtures.

Existing tests: **10 tests, zero failures/errors**. Four additional inconsistent-input cases incorrectly returned `passed=true`:

1. Change only the mission-observed receipt's `retention_id` to another ID.
2. Change only that observed receipt's `request_sha256` to a different digest.
3. Change only its `writer_owned` to `true`.
4. Replace the window's declared `binding` with `{}`.

The raw files, their manifests, and the final producer checkpoint were unchanged. The first three pass because the audit compares the observed result's status, persisted flag and receipt, but omits its own ID/SHA/writer ownership. The fourth passes because the binding comparison iterates the supplied binding keys, permitting an empty-key comparison. Required fixes are exact observed-result ID/SHA/ownership checks and an exact four-key window binding schema (`map_epoch`, `goal_token`, `inspection_request_id`, `after_stamp_ns`). This is a local validation repair, not a request to redesign the protocol.

An additional malformed top-level input `audit(None, root)` raises uncaught `AttributeError`: `consumer.get` precedes the error boundary. Return a structured failed audit for an invalid top-level object; the ordinary CLI also can load JSON `null` successfully before hitting this path.

Evidence: `frame-retention-audit-independent-01.json`, `.log`, and `review-frame-retention-audit.py` in this directory. The full per-case results retain all three accepted rows and the erroneous `passed=true`; they are not converted to regression passes. Root owns the fixes and subsequent regression evidence.

## Mission production diff recheck

Re-read production mission Node SHA `b37e82de359649c54e0dddb8ab5f211a9a53f326b7088e95535f20eaf53974ec` and Consumer SHA `ec0a3d4a42ca658a4339ad62e2bc50e92ff66e7a35139a663c3060337ea9adf6`. This is a follow-up self-review of the implementation authored by this reviewer, not an independent second-author endorsement.

The key planned gates remain present: default off; bounded constructor limits; actual settling barrier before begin_window; original exact acquisition TF and source/odom checks; wrap the actual Ledger acceptance; preserve all accepted slots; steady-timer-only successful transition; original deadline and continued pose/odom checks; no fourth-frame substitution; epoch/clock/block invalidation; normal return cancellation closes consumer selection without granting Ledger readiness; Action cancel ACK still cannot count as terminal; full history only in final result, not high-rate status. Terminal late failure updates memory without modifying an already written mission result; final source checkpoint audit remains required. Original navigation dispatch/return deadline functions are not changed by this diff.

One non-control audit clarity issue remains worth addressing: after a new Action terminal but before its settling window is opened, `inspection_snapshot()` currently nests `frame_retention.summary()` for the previous window, and that small summary lacks its binding. The new Ledger is correctly fresh and no success gate is bypassed, but the status could be read as attaching the prior window's selected/stored counts to the new goal. Include the summary's original binding and/or explicitly flag whether it matches the current Ledger/token. Do not erase the old full audit or manufacture a new window before settling.

## Repair source recheck

Root applied the local fixes. Read-only recheck of audit SHA `25b86d8372316e1297086a439f4ef57d286369d575f91ccbe9adeee60106afc0` confirms: top-level and nested consumer object checks now lie inside the bounded failure boundary; nonempty windows require exactly the four declared binding keys; observed receipts now compare retention ID, request SHA, writer ownership, reason, persisted state and receipt. These checks close the five recorded counterexamples by inspection. Test source SHA `1fc5794f935937f7b02aea3772ad488f9b0825efc7bf8125d4cacd8b01ecf6fc` adds three test methods covering all five and two additional malformed top-level shapes, for 13 total methods. This reviewer did not rerun the repaired suite; root is running the frozen full suite. The original false-pass evidence remains unchanged.
