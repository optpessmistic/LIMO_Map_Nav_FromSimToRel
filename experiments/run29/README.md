# Run29 compact diagnostic archive

Mission, navigation-recovery diagnostics, parameter readbacks and original navigation logs are retained here. Original failure/unknown values remain unchanged; CI success and a stationary stop do not imply mission success.

Runtime revision: `9e0a32687abe52eca1751a1e0e24771618835c38`; external CI receipt: `41601275268`. The actual root session exit is recorded separately in `external-receipts.json`.

**This is not a complete retained-frame archive.** RGB-D binaries and metadata remain at `outputs/ros_smoke/run29-navigation-recovery/retained`. `external-evidence.json` lists their exact paths, sizes and SHA256. MCAP/DB3, posegraph/data and full observer data are also hash-only references. No image tar or bag copy is included. Keep those original directories to rerun the strict audit; the existing audit result is preserved without reinterpretation.

See `run29-outcome-review.md` for the run28 comparison and `run29-outcome-review.json` for original mission flags, recovery started/ready, actual later goal dispatches and terminal error fields. Ready alone is not dispatch success; two runs cannot establish that cooldown caused improvement. Missing files are listed explicitly.

`runner/` contains exact small helper snapshots as `.txt`; restore original filenames in the repository before reuse. Production/test source hashes are bound by preparation and the separate navigation-recovery validation archive. No full node source or repeated validation logs are copied here.

Verify archived bytes with `sha256sum -c SHA256SUMS` from this directory. Archive-local `.gitattributes` disables text conversion. External source hashes require the originals and are not validated by that archive-only command.
