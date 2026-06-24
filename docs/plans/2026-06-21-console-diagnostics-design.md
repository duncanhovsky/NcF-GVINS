# Console Diagnostics Design

## Goal

Replace noisy terminal output with compact, default-on diagnostics that expose sensor health decisions, motion trend checks, and each optimization window.

## Design

Add a small `diagnostics` formatter module that owns text layout. `GVINS` will collect optimization-window facts before and after solving, then emit a fixed-prefix block through the existing logging macros. The block stays compact: one summary line, one before/after pose line, and one slot table with at most the configured sliding-window slots.

Health and motion-trend diagnostics stay event-oriented. Normal samples are summarized inside optimization logs; degradations, recoveries, trend outliers, and threshold reweighting are printed immediately with measured values and thresholds.

## Scope

- Remove or demote startup, raw timestamp, Ceres brief report, factor-count, and routine outlier-count logs.
- Keep errors, failed configuration/output-path checks, Ctrl-C, shutdown, and degraded/recovered events.
- Add default-on YAML diagnostics options under `nc_extension.console_diagnostics`, while preserving default-on behavior when the section is absent.

## Verification

Add a standalone formatter test that can compile without ROS. Compile/run it first as a failing test, then implement the formatter and wire it into `GVINS`.
