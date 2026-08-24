# MaixCAM2 红/蓝球 ROI 识别 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 `licang_BLUE_RED_BALL.py` 中实现 MaixCAM2 补光灯、UART/触摸模式切换、ROI 红蓝球识别、连续帧锁存和实时屏幕显示。

**Architecture:** 保持单文件实现。顶部集中放置现场参数；硬件初始化函数分别负责 B25、UART4、摄像头、显示和触摸；主循环每轮先轮询输入，再完成一帧 ROI 检测、锁存判断和 UI 绘制。纯 Python 的 `_selftest()` 不依赖 MaixPy，用假的串口/blob 覆盖协议和锁存逻辑。

**Tech Stack:** MaixPy `camera`、`display`、`image`、`gpio`、`pinmap`、`touchscreen`、`maix.peripheral.uart`；Python 语法检查使用 `py_compile`。

## Global Constraints

- 只修改 `licang_BLUE_RED_BALL.py`；设计和计划文档只记录本次实现。
- MaixCAM2 UART4：TX=A21、RX=A22、设备 `/dev/ttyS4`，115200 8N1。
- MaixCAM2 照明 LED 使用 B25，高电平开启，启动后保持开启。
- `current_mode=1` 表示红球，`current_mode=2` 表示蓝球；所有入口调用 `set_mode(mode, source)`。
- UART 输入兼容 ASCII `1`/`2`、CR、LF；识别成功统一发送 `b"1\\n"`。
- `find_blobs` 必须直接接收 `roi=ROI`，不得整幅图检测后再裁剪。
- 识别确认默认连续 3 帧，解除锁定默认连续 5 帧无目标。
- 红蓝阈值是 B25 补光灯开启后的待实机标定参数，不能宣称为最终最佳值。

## File Structure

- Modify: `licang_BLUE_RED_BALL.py` — 唯一运行脚本，包含配置、硬件适配、检测、状态和 `_selftest()`。
- Create: `docs/superpowers/plans/2026-08-24-maixcam2-red-blue-ball-roi.md` — 本实现计划。

### Task 1: Add configuration, MaixPy imports, hardware initialization, and mode protocol

**Files:**
- Modify: `licang_BLUE_RED_BALL.py`

**Interfaces:**
- `light_init() -> object`, `light_on() -> None`, `light_off() -> None`.
- `init_uart() -> object` configures A21/A22 as UART4 before creating `/dev/ttyS4`.
- `set_mode(mode: int, source: str) -> bool` validates 1/2, updates `current_mode`, and clears detection state.
- `process_command_bytes(data) -> None` accepts bytes/bytearray/string and routes only `1` and `2` to `set_mode`.

- [ ] **Step 1: Define the top-level configuration and fallback imports**

  Put `CAMERA_WIDTH=640`, `CAMERA_HEIGHT=480`, `UART_DEVICE="/dev/ttyS4"`, `UART_BAUDRATE=115200`, `ROI=[120,80,400,300]`, the two LAB threshold tuples, blob limits, confirmation/lost-frame counts, and bottom button rectangles at the top. Mark both thresholds with the required field-calibration comment. Import MaixPy modules inside a `try` block and set `MAIXPY=False` on a local development machine.

- [ ] **Step 2: Implement B25 and UART4 initialization**

  `light_init()` calls `pinmap.set_pin_function("B25", "B25")`, creates `gpio.GPIO("B25", gpio.Mode.OUT)`, and stores it. `light_on()` writes `1`; `light_off()` writes `0`. `init_uart()` maps `A21` to `UART4_TX`, maps `A22` to `UART4_RX`, then returns `uart.UART("/dev/ttyS4", 115200)`.

- [ ] **Step 3: Implement shared mode state and byte protocol**

  Initialize `current_mode=1`, `detected_latched=False`, `detected_streak=0`, and `lost_frames=0`. `set_mode` must reset all three detection fields whenever it accepts a mode. `process_command_bytes` must ignore CR/LF and unrelated bytes while accepting both `bytes` and text data.

- [ ] **Step 4: Run a syntax check before adding the remaining runtime code**

  Run `python -m py_compile licang_BLUE_RED_BALL.py`.

  Expected: exit code 0.

### Task 2: Add ROI blob detection, lockout state machine, touch input, and UI loop

**Files:**
- Modify: `licang_BLUE_RED_BALL.py`

**Interfaces:**
- `detect_ball(img, mode=None) -> blob | None` always calls `img.find_blobs([threshold], roi=ROI, pixels_threshold=PIXELS_THRESHOLD, area_threshold=AREA_THRESHOLD, merge=True)`.
- `update_detection(blob, serial) -> None` updates frame counters and sends exactly one `b"1\\n"` per latch period.
- `uart_process(serial) -> None` performs one immediate-return `serial.read()` poll.
- `touch_process(touch, image_width, image_height) -> None` handles RED/BLUE button presses through `set_mode`.
- `draw_ui(img, blob) -> None` always draws ROI and buttons, and conditionally draws target geometry and status.
- `main() -> None` initializes hardware and runs the continuous vision loop.

- [ ] **Step 1: Implement blob access and filtering helpers**

  Read `x/y/w/h`, `pixels()` and `area()` from a MaixPy blob. Reject candidates when pixel/area count is below `MIN_AREA`, width/height is outside configured min/max, or `width / height` is outside `RATIO_MIN..RATIO_MAX`. Return the largest remaining candidate by area/pixels.

- [ ] **Step 2: Implement direct ROI detection**

  Select `COLOR_THRESHOLDS[current_mode]`, call `find_blobs` with the configured ROI and thresholds, filter every returned blob, and return the largest valid one. Catch only per-frame vision exceptions so a bad frame does not terminate the main loop.

- [ ] **Step 3: Implement confirmation and release lock**

  A valid blob increments `detected_streak` and clears `lost_frames`; after 3 valid frames, send `serial.write(b"1\\n")` once and set `detected_latched=True`. A missing/invalid blob clears the positive streak and increments `lost_frames`; after 5 missing frames clear the latch. A latched state must suppress repeated sends while the target remains present.

- [ ] **Step 4: Implement non-blocking UART and touch polling**

  `uart_process` calls `serial.read()` with no blocking timeout argument, passes any returned bytes to `process_command_bytes`, and catches read errors. `touch_process` calls `touch.read()`, requires a pressed value, scales screen coordinates to the current image size, checks the two bottom button rectangles, debounces them, and routes the selected mode through `set_mode`.

- [ ] **Step 5: Implement the real-time display**

  Draw the configured ROI every frame. Draw a target rectangle and center point when a valid blob exists. Draw `MODE: RED` or `MODE: BLUE`, `STATUS: DETECTED` only after the latch is confirmed otherwise `STATUS: SEARCHING`, and visually distinguish the selected bottom button.

- [ ] **Step 6: Implement the main loop and shutdown-safe structure**

  Initialize and turn on the light before camera/display/touch/UART. In `while True`, call UART poll, touch poll, camera read, detection, lock update, UI draw, and display show in that order. Do not call a blocking UART read or sleep long enough to pause recognition; use a short `time.sleep_ms(1)` only if available for CPU yielding.

### Task 3: Add self-test and verify the complete script

**Files:**
- Modify: `licang_BLUE_RED_BALL.py`

**Interfaces:**
- `_selftest() -> None` runs without MaixPy hardware and exits successfully only if protocol, filtering, mode reset, lockout, and re-trigger behavior pass.

- [ ] **Step 1: Add fake test objects and pure logic assertions**

  Test `process_command_bytes(b"1\\r\\n2")` ends in mode 2, test invalid bytes do not change mode, test a non-square or undersized fake blob is rejected, and test the largest valid fake blob is selected.

- [ ] **Step 2: Test detection lockout and re-trigger**

  Feed three valid blobs to a fake serial and assert exactly `[b"1\\n"]`; feed more valid frames and assert no duplicate; feed five missing frames, then three valid frames, and assert the second send. Call `set_mode(1, "test")` while a target is latched and assert a new three-frame sequence sends again.

- [ ] **Step 3: Run local verification commands**

  Run:

  ```powershell
  python -m py_compile licang_BLUE_RED_BALL.py
  python licang_BLUE_RED_BALL.py --selftest
  rg -n "find_blobs|serial\.read|UART_DEVICE|ROI|B25|set_mode|detected_latched" licang_BLUE_RED_BALL.py
  git diff --check
  ```

  Expected: syntax check and self-test exit 0; `find_blobs` includes `roi=ROI`; UART read has no blocking timeout; B25, UART4, mode reset, ROI, and latch symbols are present; diff check is clean.

- [ ] **Step 4: Review against the hardware test matrix**

  Confirm the code path for: startup light ON; UART `1`/`2`; red/blue exclusivity through thresholds; touch RED/BLUE; UART overriding touch in arrival order; outside-ROI rejection; one-shot reporting; release after five lost frames; and re-arming when a new command arrives. Record that physical LED, UART wiring, touch, and color-threshold behavior require on-device testing.
