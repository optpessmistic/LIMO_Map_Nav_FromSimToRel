# Run30 compact diagnostic archive

Mission, navigation-recovery diagnostics, parameter readbacks and original navigation logs are retained here. Original failure/unknown values remain unchanged; CI success and a stationary stop do not imply mission success.

Runtime revision: `a335f4c2104b8421bd8ae249f20229c81f9f8c54`; external CI receipt: `41605303559`. The actual root session exit is recorded separately in `external-receipts.json`.

**This is not a complete retained-frame archive.** RGB-D binaries and metadata remain at `outputs/ros_smoke/run30-steady-tf/retained`. `external-evidence.json` lists their exact paths, sizes and SHA256. MCAP/DB3, posegraph/data and full observer data are also hash-only references. No image tar or bag copy is included. Keep those original directories to rerun the strict audit; the existing audit result is preserved without reinterpretation.

See `run30-outcome-review.md` for the run29 comparison and `run30-outcome-review.json` for original mission flags, recovery started/ready, actual later goal dispatches and terminal error fields. Ready alone is not dispatch success; two runs cannot establish that steady TF pacing caused improvement. Missing files are listed explicitly.

`runner/` contains exact small helper snapshots as `.txt`; restore original filenames in the repository before reuse. Production/test source hashes are bound by preparation and the separate navigation-recovery validation archive. The closed SLAM build receipt (94045), build script/log, source binding, prefix, SHA list, ldd and full nm output are retained under `validation/`; the nm text is lossless gzip, with original and archived hashes in the manifest. Build/CI success is separate from actual loaded-library identity and mission outcome. The optional run30 clock probe source/result/completion receipt are included when available; their absence is listed explicitly and their presence does not establish a causal repair. The closed-bag run29/run30 TF delivery comparison and its decoder/wrapper are also in validation/. ROS bag arrival gaps and advancing header gaps measure different things; neither reconstructs Nav2 private caches. No full node source, shared-library binary or repeated recovery validation logs are copied here.

Verify archived bytes with `sha256sum -c SHA256SUMS` from this directory. Archive-local `.gitattributes` disables text conversion. External source hashes require the originals and are not validated by that archive-only command.
