# Offline GNSS Drain Design

## Goal

Add an NcF runtime mode switch so offline bag evaluation can favor deterministic timestamp-ordered fusion over online real-time progress.

## Behavior

`nc_extension.realtime_mode` defaults to `false`.

When `realtime_mode: true`, NcF-GVINS keeps the current online behavior: IMU propagation continues, GNSS timeout can mark a recovery segment when no GNSS has been processed recently, and the estimator does not wait for lower-rate observations.

When `realtime_mode: false`, the fusion loop treats already-buffered GNSS as pending offline data before declaring a GNSS timeout. This prevents a slow optimizer or callback backlog from turning "GNSS has arrived but has not been consumed yet" into a false outage.

## Architecture

Keep the change close to the existing GNSS timeout path. Introduce a small pure helper for timeout eligibility, add a `realtime_mode_` config field, and make `checkGnssTimeout()` skip timeout when offline mode has buffered GNSS with a timestamp earlier than the current fusion time.

This is intentionally conservative: it does not redesign frame scheduling or block forever waiting for future data. It fixes the observed backlog-induced false timeout while preserving the rest of the state machine.

## Testing

Add a lightweight C++ assertion test for the timeout policy:

- realtime mode allows timeout even with a pending GNSS observation;
- offline mode suppresses timeout when an eligible GNSS observation is buffered;
- offline mode still allows timeout when no GNSS is pending.

