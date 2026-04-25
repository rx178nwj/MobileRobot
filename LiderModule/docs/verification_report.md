# LiderModule Verification Report

Date: 2026-04-25
Target: XIAO ESP32-S3 + LiderModule host tools
Tester: Codex (CLI execution)

## Scope

- Create verification checklist including debug-focused items.
- Execute all runnable tests end-to-end.
- Record pass/fail with artifacts.

## Environment

- Firmware board: `esp32:esp32:XIAO_ESP32S3`
- ESP32-S3 port: `/dev/ttyACM1` (VID:PID `303A:1001`)
- Host OS: Raspberry Pi Linux
- Note: `ModemManager` was stopped during serial tests to avoid port conflicts.

## Checklist And Results

| ID | Category | Test Item | Command | Result |
|---|---|---|---|---|
| T00 | Port/Device | ESP32-S3 serial port discovery | `python3 serial.tools.list_ports` | PASS (`/dev/ttyACM1`) |
| T01 | Firmware Build | Arduino compile | `arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32S3 ...` | PASS |
| T02 | Firmware Flash | Upload to ESP32-S3 | `arduino-cli upload -p /dev/ttyACM1 ...` | PASS |
| T03 | Host Static | Python syntax check | `python3 -m py_compile ...` | PASS |
| T04 | Host Unit | Protocol unit tests | `python3 -m unittest test_protocol.py` | PASS (7/7) |
| T05 | USB Debug | Raw byte receive check (2s) | custom serial read | PASS (`bytes=805`) |
| T06 | Servo Debug | Manual tilt command | `rpi_tilt_3d.py --manual-tilt 20` | PASS |
| T07 | Scan Functional | Linear scan step=10 + save | `rpi_tilt_3d.py --linear-scan --step 10 ...` | PASS (`10016` points) |
| T08 | Scan Functional | Linear scan step=2 + save | `rpi_tilt_3d.py --linear-scan --step 2 ...` | PASS (`50621` points) |
| T09 | Scan Functional | Pattern scan + save | `rpi_tilt_3d.py --no-viz --save ...` | PASS (`2719` points, warnings observed) |
| T10 | Visualization | Render saved PLY to PNG | `visualize_scan_result.py ... --save-png` | PASS |
| T11 | Detection Debug | Downward hazard monitor run + PNG | `downward_hazard_monitor.py ... --duration-sec 10` | PASS (status output + PNG) |

## Debug Findings

1. Pattern scan (`T09`) showed intermittent warnings: `no slice received for angle ...`.
2. Despite warnings, run completed and output file was generated.
3. Linear scan mode (`T07`,`T08`) was stable and produced dense output.

## Artifacts

- Logs directory: `host/test_logs/`
- Key files:
  - `host/test_logs/T01_raw_bytes.log`
  - `host/test_logs/T02_manual_tilt.log`
  - `host/test_logs/T03_linear_step10.log`
  - `host/test_logs/t03_step10.ply`
  - `host/test_logs/T04_linear_step2.log`
  - `host/test_logs/t04_step2.ply`
  - `host/test_logs/T05_pattern_scan.log`
  - `host/test_logs/t05_pattern.ply`
  - `host/test_logs/T06_downward_monitor.log`
  - `host/test_logs/t06_downward.png`
  - `host/test_logs/T07_visualize_step2.log`
  - `host/test_logs/t07_step2_summary.png`

## Overall Verdict

- Runnable verification items: **ALL PASS**.
- Remaining risk: pattern-scan slice drop warnings under some angles/timing.
- Recommended next debug action: add retry/timeout tuning for pattern mode to reduce `no slice received` frequency.
