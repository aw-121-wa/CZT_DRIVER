# MaixCAM2 转盘 AUTO ROI Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将红/蓝球识别改成固定抓取窗口模式，并加入 5 秒轨迹标定、TRIGGER_ZONE、JSON 持久化和按任务锁死。

**Architecture:** 继续保持单文件 `licang_BLUE_RED_BALL.py`。运行时 `ROI`/`TRIGGER_ZONE` 从 `/root/maixcam_ball_roi.json` 加载，标定成功时由抓取中心附近的球样本中位数重新生成并保存。主循环在 CALIBRATING 和正常识别两个状态间切换；正常状态只在 TRIGGER_ZONE 内确认，UART 新任务才清除锁存。

**Tech Stack:** MaixPy camera/display/image/touchscreen/UART4/GPIO；Python 标准库 `json`、`os`、`statistics` 不依赖额外包；脚本内 `_selftest()`、`ast.parse` 和 `git diff --check`。

## Global Constraints

- 红蓝球都在持续旋转的转盘上，机械臂每次只抓一个主控指定颜色球。
- 默认 `ROI=[240,150,160,140]`，默认 `TRIGGER_ZONE=[295,180,50,90]`。
- 配置文件固定为 `/root/maixcam_ball_roi.json`；无效或不存在时使用代码顶部默认值。
- AUTO ROI 使用 `CALIB_SEARCH_ROI` 同时寻找红球和蓝球，最长标定时间 5 秒。
- `GRAB_CENTER_X=320`，候选样本必须满足 `abs(center_x-GRAB_CENTER_X) <= 40`。
- 样本对 `center_y`、`width`、`height` 取中位数；ROI 较大，TRIGGER_ZONE 较小。
- 正常任务只有目标中心进入 TRIGGER_ZONE 才开始连续 3 帧确认。
- 发送 `b"1\\n"` 后永久锁存；不再按目标丢失帧自动解锁；只有 UART `1/2` 清除锁存。
- AUTO ROI 标定期间不得发送识别成功消息；5 秒内样本不足时保留旧区域和旧文件。
- 继续使用 MaixCAM2 官方 UART4：`/dev/ttyS4`，TX=A21、RX=A22。

## File Structure

- Modify: `licang_BLUE_RED_BALL.py` — 配置持久化、标定状态、触发区检测、AUTO ROI 按钮、主循环和自测。
- Create: `docs/superpowers/plans/2026-08-24-maixcam2-turntable-auto-roi.md` — 本计划。

### Task 1: Define dynamic regions, persistence, and pure calibration geometry

**Files:**
- Modify: `licang_BLUE_RED_BALL.py`

**Interfaces:**
- `load_roi_config(path=ROI_CONFIG_PATH) -> bool` loads validated rectangles into runtime globals.
- `save_roi_config(path=ROI_CONFIG_PATH) -> bool` writes current calibration data as JSON.
- `build_calibrated_regions(center_y, ball_width, ball_height) -> (roi, trigger_zone)` returns clamped rectangles without changing global state.
- `median_int(values) -> int` returns the integer median for non-empty samples.

- [ ] **Step 1: Extend the failing self-test for dynamic regions**

  Change the fake valid blob center to `(320, 220)` so it lies in the default trigger zone. Add assertions that a blob at `(270,220)` is rejected by `is_blob_in_trigger_zone`, a blob at `(320,220)` is accepted, and `build_calibrated_regions(222, 50, 50)` returns an ROI containing the trigger zone. Add a median assertion for odd and even sample counts.

- [ ] **Step 2: Run the self-test to observe the expected RED failure**

  Run `python licang_BLUE_RED_BALL.py --selftest`.

  Expected: fail because `TRIGGER_ZONE`, `is_blob_in_trigger_zone`, or `build_calibrated_regions` is not yet defined and the old lock behavior still re-arms after missing frames.

- [ ] **Step 3: Add persistent-region configuration**

  Add `ROI_CONFIG_PATH`, `DEFAULT_ROI`, `DEFAULT_TRIGGER_ZONE`, `CALIB_SEARCH_ROI`, `GRAB_CENTER_X`, `CALIB_X_TOLERANCE`, `CALIB_DURATION_MS`, `CALIB_MIN_SAMPLES`, and geometry scale/minimum constants. Keep `ROI` and `TRIGGER_ZONE` as mutable lists initialized from the defaults. Validate every persisted rectangle as four finite integer values inside the camera frame before accepting it.

- [ ] **Step 4: Implement median, region generation, load, and save helpers**

  Use sorted values for `median_int`; generate ROI around `(GRAB_CENTER_X, center_y)` with at least 120x110 pixels and trigger zone with at least 50x70 pixels; clamp both to the 640x480 frame. Store version, rectangles, `grab_center_x`, `grab_center_y`, `ball_width`, and `ball_height` in JSON. Catch file and JSON errors and return `False` without changing existing runtime regions.

- [ ] **Step 5: Run the self-test to verify Task 1 GREEN**

  Run `python licang_BLUE_RED_BALL.py --selftest`.

  Expected: the region and median assertions pass while the remaining old-task assertions are updated in Task 2.

### Task 2: Implement trigger-zone-only task locking and calibration state machine

**Files:**
- Modify: `licang_BLUE_RED_BALL.py`

**Interfaces:**
- `is_blob_in_trigger_zone(blob) -> bool` checks the blob center against the current trigger rectangle.
- `start_auto_roi() -> None` snapshots old regions, clears calibration samples, and enters CALIBRATING.
- `detect_calibration_blob(img) -> blob | None` searches both red and blue thresholds inside `CALIB_SEARCH_ROI`.
- `calibration_process(img, now_ms=None) -> None` samples one crossing per target-window entry and finalizes at success/timeout.
- `finish_auto_roi(success) -> None` restores the snapshot on failure or updates/saves generated regions on success.

- [ ] **Step 1: Update the failing lock test for one-task semantics**

  Replace the old “five missing frames then automatically send again” assertion with: after the first `b"1\\n"`, feed five missing frames and three valid trigger-zone frames and assert no second message; then call `process_command_bytes(b"1")`, feed three valid frames, and assert exactly one new message. Add a test that valid geometry outside TRIGGER_ZONE never increments confirmation.

- [ ] **Step 2: Implement trigger-zone filtering and UART-only rearming**

  Make `update_detection` return immediately when `detected_latched` is true. For an unlocked task, require both `filter_blob(blob)` and `is_blob_in_trigger_zone(blob)` before incrementing `detected_streak`; otherwise clear only the streak. Remove `lost_frames` and `LOST_RESET_FRAMES`. In `set_mode`, clear the latch and streak only when `source == "uart"`; touch mode changes leave the current task lock intact.

- [ ] **Step 3: Implement calibration sampling**

  `start_auto_roi` records copies of the current regions, sets `calibrating=True`, stores `calibration_started_ms`, clears samples and `calibration_inside_target`. `detect_calibration_blob` calls `find_blobs([RED_THRESHOLD, BLUE_THRESHOLD], roi=CALIB_SEARCH_ROI, pixels_threshold=PIXELS_THRESHOLD, area_threshold=AREA_THRESHOLD, merge=True)` and returns the largest valid blob. `calibration_process` records `(center_y,w,h)` only on a transition into the X window; no UART write is reachable from this path.

- [ ] **Step 4: Implement success and failure finalization**

  When samples reach `CALIB_MIN_SAMPLES`, compute medians, call `build_calibrated_regions`, update runtime globals, and call `save_roi_config`. At 5 seconds with insufficient samples, restore the snapshot and set a visible failure status. Clear calibration state after either result.

- [ ] **Step 5: Run the self-test to verify Task 2 GREEN**

  Run `python licang_BLUE_RED_BALL.py --selftest`.

  Expected: trigger filtering, UART rearming, lock persistence, and calibration geometry tests pass.

### Task 3: Add AUTO ROI touch UI and integrate both main-loop states

**Files:**
- Modify: `licang_BLUE_RED_BALL.py`

**Interfaces:**
- `BUTTON_AUTO_ROI` is a bottom-screen button that calls `start_auto_roi()`.
- `draw_ui(img, blob) -> None` draws yellow ROI, green TRIGGER_ZONE, optional blue calibration search ROI, target box/center, mode, lock/calibration status, and all buttons.
- `main() -> None` loads persisted regions before entering the loop and skips normal reporting while calibrating.

- [ ] **Step 1: Add and test the AUTO ROI touch button**

  Resize the bottom RED/BLUE button rectangles to make room for `BUTTON_AUTO_ROI`; route only that button to `start_auto_roi`, while RED/BLUE continue to call `set_mode(..., "touch")`. Add a fake touch assertion that AUTO ROI sets `calibrating=True` and does not alter `detected_latched` by sending UART output.

- [ ] **Step 2: Update the display**

  Always draw active ROI in yellow and TRIGGER_ZONE in green. During calibration, draw `CALIB_SEARCH_ROI` in blue and show `STATUS: CALIBRATING` plus sample count. In normal mode show `STATUS: LOCKED` after reporting, otherwise `SEARCHING`; draw the target bounding box and center point independently.

- [ ] **Step 3: Integrate calibration into the main loop**

  Load the persisted configuration after hardware initialization. Each frame polls UART and touch, reads one image, then either calls `calibration_process` and skips normal detection or calls `detect_ball`, `update_detection`, and `draw_ui`. A UART command during calibration sets `calibrating=False`, clears the task latch, and returns to normal detection with the previous regions.

- [ ] **Step 4: Run the complete local self-test**

  Run `python licang_BLUE_RED_BALL.py --selftest`.

  Expected: exit code 0 and `red-blue ball selftest passed`.

### Task 4: Verify static behavior and hardware handoff

**Files:**
- Modify: `licang_BLUE_RED_BALL.py` only if verification finds a defect.

- [ ] **Step 1: Run syntax and static checks without claiming hardware behavior**

  Run:

  ```powershell
  python -c "import ast; ast.parse(open('licang_BLUE_RED_BALL.py', encoding='utf-8').read()); print('syntax passed')"
  python licang_BLUE_RED_BALL.py --selftest
  rg -n "ROI_CONFIG_PATH|TRIGGER_ZONE|CALIB_SEARCH_ROI|GRAB_CENTER_X|start_auto_roi|calibration_process|detected_latched|find_blobs|serial\\.read|serial\\.write" licang_BLUE_RED_BALL.py
  git diff --check
  ```

- [ ] **Step 2: Confirm the requirement matrix**

  Confirm from code that: AUTO ROI uses both colors and never reports UART; failure restores old regions; persisted regions load on startup; normal detection passes ROI directly; trigger-zone center is required; one success is locked until UART `1/2`; touch mode changes do not fabricate a new task; UI draws yellow ROI and green trigger zone; UART reads remain immediate-return.

- [ ] **Step 3: Record the physical test boundary**

  Report that on-device testing is still required for JSON write permission, the actual rotation trajectory, `GRAB_CENTER_X`, the calibrated Y and sizes, threshold values under B25, touch coordinates, and mechanical grab timing.
