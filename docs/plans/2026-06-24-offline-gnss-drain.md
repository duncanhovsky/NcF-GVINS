# Offline GNSS Drain Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add `nc_extension.realtime_mode` so offline bag evaluation does not falsely trigger GNSS timeout while GNSS observations are buffered but not yet consumed.

**Architecture:** Add a small timeout policy helper under `ic_gvins/health`, cover it with a standalone C++ test, then wire it into `GVINS::checkGnssTimeout()`. Parse `realtime_mode` from YAML with default `false`, and update the main configs and README.

**Tech Stack:** C++14, YAML config, existing assert-style C++ tests.

---

### Task 1: Timeout Policy Test

**Files:**
- Create: `ic_gvins/tests/test_gnss_timeout_policy.cc`
- Create: `ic_gvins/ic_gvins/health/gnss_timeout_policy.h`

**Step 1: Write the failing test**

Create a test that includes `ic_gvins/health/gnss_timeout_policy.h` and asserts:

```cpp
assert(nc_health::shouldTriggerGnssTimeout(true, 105.0, 100.0, 2.0, true));
assert(!nc_health::shouldTriggerGnssTimeout(false, 105.0, 100.0, 2.0, true));
assert(nc_health::shouldTriggerGnssTimeout(false, 105.0, 100.0, 2.0, false));
```

**Step 2: Run test to verify it fails**

Run: `g++ -std=c++14 -I . -I ic_gvins ic_gvins/tests/test_gnss_timeout_policy.cc -o scripts/test_gnss_timeout_policy.exe`

Expected: FAIL because the header/function does not exist.

**Step 3: Write minimal implementation**

Add the helper:

```cpp
namespace nc_health {
inline bool shouldTriggerGnssTimeout(bool realtime_mode, double fusion_time,
                                     double last_processed_gnss_time,
                                     double gnss_timeout,
                                     bool has_pending_gnss_before_fusion_time) {
    if (gnss_timeout <= 0.0 || last_processed_gnss_time <= 0.0) {
        return false;
    }
    if (!realtime_mode && has_pending_gnss_before_fusion_time) {
        return false;
    }
    return (fusion_time - last_processed_gnss_time) > gnss_timeout;
}
} // namespace nc_health
```

**Step 4: Run test to verify it passes**

Run the same `g++` command, then run `scripts/test_gnss_timeout_policy.exe`.

Expected: PASS with exit code 0.

### Task 2: Wire Runtime Mode

**Files:**
- Modify: `ic_gvins/ic_gvins/ic_gvins.h`
- Modify: `ic_gvins/ic_gvins/ic_gvins.cc`
- Modify: `config/gvins*.yaml`
- Modify: `config/i2nav_ablation/*.yaml`
- Modify: `NcF-GVINS_README.md`

**Step 1: Add config field**

Add `bool realtime_mode_{false};` to `GVINS` and parse `nc_extension.realtime_mode` with default `false`.

**Step 2: Wire timeout policy**

In `checkGnssTimeout()`, compute whether a buffered GNSS exists before `fusion_time`, then call `nc_health::shouldTriggerGnssTimeout()`.

**Step 3: Update configs and docs**

Add `realtime_mode: false` to NcF configs near `enabled`, and document that offline bag evaluation should leave it false while real-time online use can set it true.

**Step 4: Verify**

Compile/run `test_gnss_timeout_policy`, then run targeted text checks for the config/docs entries.

