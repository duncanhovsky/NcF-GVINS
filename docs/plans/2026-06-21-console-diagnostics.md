# Console Diagnostics Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Build default-on, readable NcF-GVINS console diagnostics for sensor health, motion consistency, and optimization windows.

**Architecture:** Add a header-only diagnostics formatter plus lightweight structs so it can be tested without ROS/Ceres. `GVINS` populates snapshots around the two-pass optimizer and logs one compact block per optimization. Existing noisy logs are removed or replaced by event diagnostics.

**Tech Stack:** C++14, Eigen-compatible project style, glog macros already wrapped by `common/logging.h`, YAML config.

---

### Task 1: Formatter Test

**Files:**
- Create: `NcF-GVINS/ic_gvins/tests/console_diagnostics_test.cc`
- Create: `NcF-GVINS/ic_gvins/ic_gvins/diagnostics/console_diagnostics.h`

**Steps:**
1. Write a small test that builds a health line and optimization window block.
2. Compile it with `g++ -std=c++14 -I NcF-GVINS/ic_gvins`.
3. Confirm it fails before the header exists.
4. Implement the formatter.
5. Re-run the test and confirm it passes.

### Task 2: GVINS Integration

**Files:**
- Modify: `NcF-GVINS/ic_gvins/ic_gvins/ic_gvins.h`
- Modify: `NcF-GVINS/ic_gvins/ic_gvins/ic_gvins.cc`

**Steps:**
1. Add diagnostics configuration members with default enabled.
2. Parse optional `nc_extension.console_diagnostics`.
3. Capture pre/post poses, GNSS chi2/reweight results, visual outlier count, heading count, and per-slot factor counts.
4. Replace routine optimization logs with one diagnostics block.

### Task 3: ROS Noise Cleanup

**Files:**
- Modify: `NcF-GVINS/ic_gvins/ROS/fusion_ros.cc`

**Steps:**
1. Remove routine startup/raw-time console messages.
2. Keep configuration failures, SIGINT, and shutdown as warning/error logs.
3. Verify no high-rate raw timestamp log remains.

### Task 4: Build Checks

**Commands:**
- `g++ -std=c++14 -I NcF-GVINS/ic_gvins NcF-GVINS/ic_gvins/tests/console_diagnostics_test.cc -o <temp-test>`
- If ROS/catkin is available, build `ic_gvins`.
