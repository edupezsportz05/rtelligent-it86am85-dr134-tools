# Rtelligent IT86AM85 / IR86 + USR-DR134 Tools

A Python toolkit for testing and controlling Rtelligent IT86/IR86 integrated stepper drives. It includes two scripts for a USR-DR134 RS485-to-Ethernet converter and one interactive tool for a direct USB-to-RS485 connection.

> **Status: bench tools / work in progress.** These scripts are not safety-rated motion-control software. Validate all motion with the robot secured, drive wheels raised, and an accessible emergency stop before using them on the floor.

## Repository contents

| File | Purpose | Connection | Moves motors? |
|---|---|---|---|
| `dr134_motor_move.py` | Move one or more drives by a requested angle; includes a communication-only check mode | DR134 TCP, default `192.168.169.5:1502`; RTU-over-TCP by default | Yes, unless `--check` is used |
| `dr134_position_test.py` | Issue a fixed pulse-distance move to one drive and log registers 8–9 before/during/after | DR134 TCP, default `192.168.169.5:1502` | Yes |
| `it86_usb_rs485_tool.py` | Interactive menu for status, configuration, jog, angle moves, encoder monitor and register dump | Direct USB–RS485 serial COM port | Yes, depending on menu option |

The two DR134 scripts use only the Python standard library. The interactive serial tool requires `pyserial`.

## Hardware and communication setup

The current bench setup reflected in these scripts is:

- Rtelligent IT86AM85 / IR86 integrated stepper drives, Modbus slave IDs `1` and `2`.
- USR-DR134 RS485-to-Ethernet converter at `192.168.169.5`, TCP port `1502`.
- Modbus RTU frames carried through the DR134 TCP connection (transparent/RTU framing). `dr134_motor_move.py` also has an optional `--framing tcp` mode if the converter is configured for Modbus TCP.
- Serial settings recorded in the tool/manual notes: `115200`, 8 data bits, no parity, 1 stop bit (`8N1`). Baud rate, framing mode, slave IDs, wiring and converter settings must match the actual hardware.
- Register `111` read as `4000` pulses/revolution on the tested drives.

The IP address is a private-network bench default, not a universal DR134 address. Change it for your network. Verify settings on the actual hardware before any movement.

## Requirements

Use Python 3.8 or newer. On Windows, run these commands from PowerShell or Command Prompt in the repository directory.

```powershell
python --version
```

For the USB–RS485 interactive tool only:

```powershell
python -m pip install pyserial
```

The two DR134 scripts need no third-party packages.

## Quick start

### 1. Check communication without movement

```powershell
python dr134_motor_move.py --check
```

To specify the current bench address explicitly:

```powershell
python dr134_motor_move.py --host 192.168.169.5 --port 1502 --ids 1,2 --check
```

If a drive does not answer, do not proceed to movement. Check drive power/alarms, RS485 A/B wiring, slave IDs, DIP switches, baud rate, converter settings and TCP reachability.

### 2. Move by angle

The movement tool's default is set to **180 degrees**, corrected to match its help text. Read the proposed movement carefully before confirming.

```powershell
# One drive, 90 degrees forward
python dr134_motor_move.py --ids 2 --deg 90

# One drive, 90 degrees reverse
python dr134_motor_move.py --ids 2 --deg -90

# Both drives, 180 degrees; confirm at the prompt
python dr134_motor_move.py --ids 1,2 --deg 180
```

Defaults are 20 RPM, acceleration setting 10, current 1500 mA, and open-loop mode (`--mode 0`). These are script defaults, not a guarantee of suitability for every load. Avoid `--yes` during commissioning because it skips the movement confirmation prompt.

### 3. Fixed-pulse position test

Open `dr134_position_test.py` and verify these constants:

```python
SLAVE_ID = 2
TEST_PULSES = 4000
DIRECTION = "forward"
```

For a new drive/setup, start with `TEST_PULSES = 1000`; increase to `4000` only when the mechanism and available travel are safe. Then run:

```powershell
python dr134_position_test.py
```

The program writes the requested distance to registers `73` and `74`, commands register `18` (`1` forward or `2` reverse), logs registers `8–9`, and attempts command `6` (stop) after the target delta, timeout or interruption. It also attempts a stop if the movement-command response times out. A failed connection can still prevent the stop from reaching the drive, so an independent emergency stop is essential.

### 4. Interactive USB–RS485 tool

Connect a USB–RS485 adapter directly to the drive bus and run:

```powershell
python it86_usb_rs485_tool.py
```

The default COM port is `COM8`; enter the actual port when prompted. The menu supports status information, configuration writes, speed/acceleration/current settings, forward/reverse jog, angle/pulse moves, live encoder monitoring, stop, register dump, dual-drive jog and save-to-flash. Some actions change persistent drive settings. Review prompts and record original settings before writing configuration or saving it to flash.

**This program uses a serial COM port. It does not connect through the DR134 Ethernet socket.**

## Register notes

Descriptions below reflect the scripts and the supplied IT86/IR86 manual notes. Confirm them against the exact drive firmware/manual revision before adapting the code.

| Register | Meaning in this toolkit | Notes |
|---:|---|---|
| `0` | Alarm flags | Check for active alarms before motion. |
| `1` | Status flags | Includes status bits such as moving (`MOV`) in the interactive tool. |
| `8–9` | Internal-pulse absolute position | Read low word at 8 and high word at 9; combined value is `low + (high << 16)`. Fixed-pulse tests showed deltas matching requested command pulses. Physical feedback must still be validated. |
| `10` | Command speed | RPM field, according to the tool notes. |
| `11` | Bus voltage | Interactive tool scales raw value by `1/100` to volts. |
| `12–13` | Closed-loop tracking error | Signed 32-bit value in the interactive tool. |
| `17` | Command source | Scripts use `0` for internal pulse source. |
| `18` | Motion command | `1` forward fixed-length, `2` reverse fixed-length, `3` forward jog, `4` reverse jog, `6` stop, as used by these scripts. |
| `20` | Application mode | Scripts use `0` for communication control. |
| `22` | Run mode | Tool uses `0` open-loop and `1` servo. |
| `40` | Encoder resolution | Interactive tool expects `16384` counts/revolution for this configuration. |
| `47` | Encoder A/B swap | Do not change without confirming the reason and retesting. Earlier diagnostics showed `200`, despite a documented `[0,1]` range; this discrepancy remains unresolved. |
| `48` | Single-turn encoder count | Earlier bench logs showed it staying at `600` while registers `8–9` changed. Do not assume it is valid live feedback until investigated. |
| `70`, `71`, `75`, `76` | Acceleration/deceleration settings | Written by the DR134 movement tool. |
| `72`, `77` | Speed settings | Written by the DR134 movement tool. |
| `73–74` | Move distance in pulses | Low word at `73`, high word at `74`. |
| `78` | Incremental move setting | Scripts use `0`. |
| `90` | Save settings to flash | The interactive tool's save option writes `1`; this can persist changes. |
| `110` | Current setting, mA | Written by the movement/configuration tools. |
| `111` | Pulses per revolution | Read as `4000` in reported tests; treat as the configured move-command scale until independently verified. |

## Bench test report

The following findings come from the logs collected during this project.

### Communication

- TCP connection to `192.168.169.5:1502` succeeded.
- Modbus RTU-over-TCP reads succeeded for registers `8–9` and register `111`.
- For slave ID `2`, registers `8–9` returned `[5969, 14]`, combined as `14 × 65536 + 5969 = 923473`.
- Register `111` returned `4000`.

### Fixed-pulse movement tests

| Drive | Requested pulses | Initial position | Final position | Observed delta |
|---|---:|---:|---:|---:|
| Slave `1` | 1,000 | 918,763 | 919,763 | +1,000 |
| Slave `2` | 1,000 | 922,473 | 923,473 | +1,000 |
| Slave `2` | 4,000 | 923,473 | 927,473 | +4,000 |

For the 4,000-pulse test, the movement command was acknowledged, the position delta reached `+4000`, the stop command was acknowledged, and the final position remained `927473`. This supports using registers `8–9` as a position-change signal for commanded moves on this setup.

### Limitations for odometry

The exact match between commanded pulses and registers `8–9` is encouraging, but **it does not alone prove that the register is independent physical encoder feedback**. If it represents internal commanded position, a stalled motor could still produce misleading odometry. Earlier logs showed register `48` stuck at `600` during motion, so the encoder report also needs investigation.

Before using these values for odometry:

1. Mark a wheel and verify the physical rotation caused by `4000` pulses. Do not infer a full revolution solely from register `111`.
2. Repeat in reverse and check the sign of the position delta.
3. Repeat on both slave IDs and establish each wheel's left/right sign convention.
4. Check for missed steps or stalls under representative load.
5. Calibrate effective rolling diameter and measure the distance between the left and right wheel contact lines.

If `4000` pulses are verified to equal one wheel revolution and the effective wheel diameter is `160 mm`, the expected travel is `π × 0.16 m ≈ 0.503 m` per revolution, or approximately `0.126 m` per 1,000 pulses. This is a conditional estimate, not a calibrated measurement.

For differential-drive odometry, with left/right wheel travel `ΔsL`, `ΔsR` and track width `b`:

```text
Δs     = (ΔsR + ΔsL) / 2
Δtheta = (ΔsR - ΔsL) / b
```

Use a validated count-to-distance conversion and handle signed deltas, direction convention, rollover, sampling gaps and initialization of the position baseline.

## Safety

- Secure and raise the robot's drive wheels for initial tests; support the chassis mechanically.
- Keep hands, clothing, tools and cables away from rotating parts.
- Use low speed and gentle acceleration during commissioning. The supplied tool notes warn that aggressive deceleration can raise DC-bus voltage when no braking resistor is fitted.
- Stop if an alarm appears, the mechanism binds, communication becomes unreliable or the drive behaves unexpectedly.
- A Python stop command is **not** a safety-rated emergency stop. Keep a physical emergency stop or safe means of removing drive power accessible.
- Do not run two control programs simultaneously against the same drive.
- Change one parameter at a time; record original values before writing configuration or saving to flash.

## Troubleshooting

| Symptom | Checks |
|---|---|
| TCP connection fails | Verify PC/DR134 network, IP/port, power and that the converter is listening on port `1502`. |
| TCP works but Modbus does not reply | Confirm RTU-transparent vs Modbus-TCP framing, serial `115200 8N1`, RS485 A/B wiring, slave ID and drive power. Try `--framing tcp` only if the converter is configured for Modbus TCP. |
| Register write times out | Do not blindly retry a motion write; the drive may have received it even if the reply was lost. Check physical motion and use the safe stop procedure. |
| Registers `8–9` change but `48` stays fixed | Treat encoder reporting as unresolved and validate register behavior against the manual/firmware and a physical wheel-rotation test. |
| Register `47` is outside its documented range | Do not write it as a workaround. Verify address offset, firmware behavior and read implementation. |
| Motor does not move | Check alarms, control-source/application settings, move distance, run mode, enable/configuration, wiring and drive status. |

## Recommended next additions

The current package contains the three Python scripts and this README. To make the repository easier to reproduce, review and extend, add the following items in roughly this order:

1. **`docs/`** — Add a connection block diagram, DR134 network and serial configuration, RS485 wiring details, verified drive connector pinout, DIP-switch settings and the exact drive/manual firmware revision. Mark any pinout or configuration that has not yet been verified as unconfirmed.
2. **`logs/`** — Add sanitized known-good logs with date, drive firmware, slave ID, connection mode, motion settings and test conditions. Remove personal paths, network details that should not be public, and other sensitive information before committing.
3. **Wheel calibration record** — For each motor, record commanded pulses, measured physical rotations, position-register delta, direction sign, effective rolling diameter/circumference and left-to-right track width. Include the method used to measure the physical rotation.
4. **ROS 2 integration** — Once the position reading is proven to represent actual motion under load, add an odometry node, launch file, parameter YAML, and instructions for publishing `/odom` and the `odom -> base_link` transform. Document units, wheel signs, sampling rate and any rollover handling.
5. **Dependencies and basic tests** — The two DR134 scripts currently use the Python standard library; the USB–RS485 tool requires `pyserial`. Add a `requirements.txt` if dependencies expand, plus a small `tests/` or test checklist for CRC, register decoding and position-delta calculations.
6. **Demonstration media** — Add a short video or a few photos showing the guarded bench setup and a test run. Include captions and test conditions; avoid showing exposed moving components without appropriate guarding.

A possible repository layout once these additions exist:

```text
rtelligent-it86am85-dr134-tools/
├── README.md
├── LICENSE                 # MIT License (original project code/docs only)
├── dr134_motor_move.py
├── dr134_position_test.py
├── it86_usb_rs485_tool.py
├── docs/
├── logs/                   # sanitized test output only
└── tests/                  # optional software/calculation tests
```

## License and reuse

This repository includes the **MIT License**, with the copyright notice `Copyright (c) 2026 EDWIN BENNY`. Subject to the license conditions, recipients may use, copy, modify, merge, publish, distribute, sublicense and sell copies of the licensed Software. The copyright notice and permission notice must accompany copies or substantial portions of the Software. The Software is provided without warranty; see the full text in [`LICENSE`](LICENSE).

The license applies to the original project code and documentation that the copyright holder has the right to license. It does **not** automatically relicense Rtelligent or USR manufacturer manuals, third-party images, diagrams, libraries or other third-party material. Such material remains subject to its own copyright and license terms. Make sure any third-party content you add has terms that permit its inclusion.
