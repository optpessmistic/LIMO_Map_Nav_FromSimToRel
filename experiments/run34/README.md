# Run34 closed result

Runtime: `c411fcbe86c1c27ce302593b73fba405a603e002`. Root `22438` exited **1** after 602.693 s. See [outcome](analysis/run34-outcome-review.md), [full evidence summary](analysis/run34-outcome-review.json) and [completion receipt](analysis/run34-root-completion.json).

Initial survey passed; frontier navigation succeeded **4/4**. The mission then blocked on `inspection_wall_timeout_incomplete`. Mission success, return, map completion and visual completion remain false. The observation queue was not created: its denominator is **null**, not 0/12. One target annotation is not a recall score.

The consumer records **9 accepted / 9 stored pairs**. Strict retention audit remains **failed**, with **0 accepted / 0 verified** and `consumer_disabled_or_faulted`; consumer fault is `mission_blocked`. This is not a successful 0/0 audit.

Map export and terminal stationary tail passed, without establishing exploration completion or return home. Owned groups, nested recorder and writer closed; global process absence remains uncertain. No selection/bag analysis or next experiment was started.

[Copy verification](archive-validation.json) passed for 108 raw/gzip payloads. [External inputs](external-inputs.json) preserves size/SHA256 for 57 excluded MCAP, posegraph/data and retained-frame files (45,048,526 bytes). Gzip files restore exact original bytes. Verify `SHA256SUMS` with `sha256sum -c SHA256SUMS`; it excludes itself. Work stops after this round as requested.
