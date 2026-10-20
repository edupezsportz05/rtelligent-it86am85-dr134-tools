#!/usr/bin/env python3
"""
==============================================================================
 Rtelligent IT86 / IR86 integrated stepper - configure, test, jog, encoder tool
==============================================================================
Talks Modbus RTU (function codes 03 and 06) over RS485. Written against the
IR86/IT86 User Manual V4.2 and tuned for a drive running WITHOUT a braking
resistor (everything defaults to very low speed / acceleration).

------------------------------------------------------------------------------
 1. REQUIREMENTS
------------------------------------------------------------------------------
  * Python 3.8+       * pip install pyserial
  * USB-RS485 adapter (COM port), A/B wired to the drive's RJ45 port.
  * 24-50 VDC supply on VDC/GND (polarity matters!). Do not exceed 50 V.

 Run from a normal terminal for the best experience:
        python it86_usb_rs485_tool.py
 (In IDLE it works too, but "press any key to stop" does not; use the jog
  duration limit or Ctrl+C instead.)

------------------------------------------------------------------------------
 2. COMMUNICATION SETTINGS (DIP switches on the drive, power-cycle after change)
------------------------------------------------------------------------------
  Format is fixed: 8 data bits, 1 stop bit, no parity.
  Baud  (SW6,SW7):  9600=ON,ON  19200=OFF,ON  38400=ON,OFF  115200=OFF,OFF
  Slave ID (SW1-5): ON = 0, OFF = 1.   ID = SW1 + 2*SW2 + 4*SW3 + 8*SW4 + 16*SW5
        ID 1 = OFF,ON,ON,ON,ON      ID 2 = ON,OFF,ON,ON,ON
        ID 3 = OFF,OFF,ON,ON,ON     ID 31 = all OFF
  SW8: motor direction (flips rotation direction).
  The script asks for COM port, baud and slave ID when it starts. If the drive
  does not answer, it offers to scan IDs 1-31.

------------------------------------------------------------------------------
 3. WHAT EACH MENU ITEM DOES
------------------------------------------------------------------------------
  1  Device info / status   Drive ID, firmware, bus voltage, decoded alarm and
                            status bits.
  2  Configure              Writes the manual's recommended parameters, reads
                            each back (verify), and optionally saves to flash
                            (reg 90). After SAVING, POWER-CYCLE the drive so
                            lock-shaft / identification settings take effect.
  3  Settings               Run mode (open-loop / servo), jog speed, accel,
                            current. Defaults are deliberately very low.
  4  Jog forward / reverse  Continuous rotation with live encoder readout.
  5  Move by angle          Incremental move by N degrees (or pulses) and then
                            compares commanded angle vs. measured encoder angle.
  6  Live encoder monitor   Streams encoder data. Turn the shaft by hand to see
                            it respond. Press any key (terminal) or Ctrl+C.
  7  Stop motor             Decelerating stop (reg 18 = 6).
  8  Register dump          Prints the key registers.
  9  Save to flash          Reg 90 = 1.
  d  Dual-drive jog         Runs two slave IDs together (e.g. 1 and 2) at positive
                            velocity and shows both encoders live.
  h  Help                   Prints this guide.
  (Choosing any option 1-9 or s first shows that option's instructions;
   press Enter to continue or type c to cancel.)
  q  Quit                   Stops the motor and closes the port.

------------------------------------------------------------------------------
 4. ENCODER DATA - HOW TO READ IT
------------------------------------------------------------------------------
  Reg 48  single-turn encoder count, 0 .. (resolution-1). Resolution is reg 40
          (4096-line encoder x4 = 16384 counts/rev).
  The script UNWRAPS reg 48 in software, so you also see:
    enc_total   cumulative counts since the monitor/jog started (signed)
    angle(deg)  enc_total / resolution * 360
    revs        angle / 360
    enc_rpm     speed measured from the encoder (independent of the command)
  Reg 12/13  closed-loop tracking error (signed 32-bit, in encoder counts).
  Reg 10     commanded speed (RPM).   Reg 11  bus voltage (10 mV units).
  Reg 8/9    internal-pulse absolute position (on some firmware this does not
             change - rely on the encoder values instead).
  Unwrapping needs less than half a revolution of motion between samples, so
  the tool keeps polling fast (~10-20 Hz). Stay below ~150 RPM for reliable
  readings. If encoder counts move the OPPOSITE way to your commanded
  direction, either flip SW8 or write reg 47 = 1 (swaps encoder A/B) - but
  only change one of them, then re-test.

------------------------------------------------------------------------------
 5. SAFETY (no braking resistor fitted)
------------------------------------------------------------------------------
  * Fast deceleration pumps energy back into the supply -> over-voltage alarm.
    Defaults: accel/decel = 10 r/s^2 (manual minimum), speed = 20 RPM.
  * Every motion watches the drive's alarm register and bus voltage. If an
    alarm appears or the bus rises more than 5 V above its starting value, the
    script sends a decelerating stop.
  * Keep hands, hair and clothing away from the shaft. To kill everything
    instantly, cut the 24 V supply.
  * Raising speed above 100 RPM asks for confirmation. If you need higher
    speeds or heavy loads, fit a braking resistor on RB+/RB-.

------------------------------------------------------------------------------
 6. TROUBLESHOOTING QUICK LIST
------------------------------------------------------------------------------
  No reply            -> wrong COM/baud/ID, A/B swapped, DIP changed without
                         power-cycle. Use the scan offer.
  LED solid green     -> drive not enabled.   LED slow-flashing green -> OK.
  Green + red flashes -> alarm: 1 red overcurrent, 2 overvoltage, 3 internal
                         voltage, 4 encoder out of tolerance, 6 parameter
                         error, 7 motor phase loss.
  Won't move          -> run Configure, save, power-cycle, then jog in
                         open-loop first, then try servo mode.
==============================================================================
"""
import sys
import time
import struct

try:
    import serial
except ImportError:
    print("pyserial is missing. Install it with:  pip install pyserial")
    sys.exit(1)

try:
    import msvcrt          # Windows only: lets us stop a jog with any key
except ImportError:
    msvcrt = None

# ----------------------------------------------------------------------------
# Safe defaults (change here or through menu 3)
# ----------------------------------------------------------------------------
DEFAULT_PORT = "COM8"
DEFAULT_BAUD = 115200
DEFAULT_SPEED_RPM = 20       # very low
DEFAULT_ACCEL = 10           # r/s^2, manual minimum
DEFAULT_CURRENT_MA = 1500
BUS_RISE_LIMIT_V = 5.0       # abort if bus rises this far above starting value
SPEED_CONFIRM_ABOVE = 100    # ask for confirmation above this RPM
SPEED_MAX = 300

# Parameters written by "Configure" (V4.2 manual defaults).
# Pole pairs (39) and out-of-tolerance threshold (45) are intentionally NOT
# written: on this firmware they did not hold the written value.
CONFIG_PARAMS = [
    (17, 0, "command source = internal pulse"),
    (20, 0, "application = communication control"),
    (26, 50, "standby current %"),
    (27, 500, "standby time ms"),
    (28, 512, "pulse filter"),
    (30, 1, "auto PI on (identifies motor at power-up)"),
    (33, 1000, "user R mOhm"),
    (34, 1, "user L mH"),
    (35, 200, "torque constant"),
    (38, 256, "current loop KC"),
    (40, 16384, "encoder resolution (4096 lines x4)"),
    (41, 0, "positioning detect mode"),
    (42, 10, "positioning accuracy"),
    (43, 50, "positioning duration"),
    (44, 100, "positioning start time"),
    (46, 0, "out-of-tolerance threshold high word"),
    (78, 0, "incremental motion"),
    (89, 0, "no auto-home at power-up"),
    (105, 16000, "lock-shaft ramp time"),
    (106, 4000, "lock-shaft duration"),
    (111, 4000, "pulses per revolution"),
    (113, 30, "current compensation %"),
    (114, 30, "current-loop gain compensation %"),
    (115, 0, "phase-loss detection off"),
    (116, 50, "phase-loss threshold"),
    (117, 0, "stall detection off"),
    (118, 180, "stall start speed"),
    (119, 2, "stall error threshold"),
    (130, 300, "velocity filter 1"),
    (131, 900, "velocity filter 2"),
    (132, 2000, "acceleration filter"),
    (150, 3500, "position Kp"),
    (151, 0, "position Kd"),
    (152, 40, "velocity Kp"),
    (153, 100, "velocity Ki"),
    (154, 0, "acceleration feedforward"),
    (155, 3000, "position output filter"),
    (156, 3000, "position differential filter"),
    (157, 500, "speed output filter"),
    (158, 80, "velocity feedback %"),
]

DUMP_REGS = [
    (0, "alarm flags"), (1, "status flags"), (10, "command speed rpm"),
    (11, "bus voltage (x10 mV)"), (17, "command source"), (20, "application mode"),
    (22, "run mode (0 open, 1 servo)"), (23, "direction (SW8)"),
    (24, "microstep (display)"), (25, "max current (display)"),
    (30, "auto PI"), (31, "identified R mOhm"), (32, "identified L mH"),
    (39, "pole pairs"), (40, "encoder resolution"), (47, "encoder A/B swap"),
    (48, "encoder single-turn count"), (93, "drive ID"), (94, "firmware version"),
    (105, "lock ramp"), (106, "lock time"), (110, "current mA"),
    (111, "pulses/rev"), (115, "phase-loss detect"), (117, "stall detect"),
]

ALARM_BITS = {8: "ECDE1 encoder fault", 7: "POSE tracking error",
              6: "MPE phase loss", 5: "MEM parameter error", 4: "OT over-temp",
              3: "UV under-voltage", 2: "OV over-voltage", 1: "OC over-current",
              0: "IVE internal voltage"}
STATUS_BITS = {11: "TC", 10: "STALL", 9: "NL", 8: "PL", 7: "CLAMP", 6: "ARRSPD",
               5: "RDY", 4: "HOME", 3: "MOV", 2: "INPOS", 1: "ALM", 0: "ENA"}


# ----------------------------------------------------------------------------
# Small helpers
# ----------------------------------------------------------------------------
def crc16(data):
    c = 0xFFFF
    for x in data:
        c ^= x
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return struct.pack("<H", c)


def s16(v):
    return v - 65536 if v > 32767 else v


def s32(lo, hi):
    v = (hi << 16) | lo
    return v - (1 << 32) if v & 0x80000000 else v


def decode_alarm(v):
    names = [n for b, n in ALARM_BITS.items() if v & (1 << b)]
    return ", ".join(names) if names else "no alarms"


def decode_status(v, short=False):
    names = [n for b, n in STATUS_BITS.items() if v & (1 << b)]
    return " ".join(names) if names else "-"


def ask(prompt, default=None):
    suffix = f" [{default}]" if default is not None else ""
    val = input(f"{prompt}{suffix}: ").strip()
    return val if val else (str(default) if default is not None else "")


def ask_int(prompt, default, lo, hi):
    while True:
        raw = ask(prompt, default)
        try:
            v = int(raw)
            if lo <= v <= hi:
                return v
        except ValueError:
            pass
        print(f"  Enter a whole number from {lo} to {hi}.")


def ask_float(prompt, default):
    while True:
        try:
            return float(ask(prompt, default))
        except ValueError:
            print("  Enter a number.")


def key_pressed():
    if msvcrt and msvcrt.kbhit():
        msvcrt.getch()
        return True
    return False


# ----------------------------------------------------------------------------
# Modbus driver
# ----------------------------------------------------------------------------
class Drive:
    def __init__(self, port, baud, slave, ser=None):
        self.slave = slave
        self.owns_port = ser is None
        self.ser = ser if ser is not None else serial.Serial(
            port, baud, bytesize=8, parity="N", stopbits=1, timeout=0.4)
        self.last_exception = None
        self.last_enc = 0

    def clone(self, slave):
        """Another drive on the SAME RS485 bus / COM port (used by dual mode)."""
        return Drive(None, None, slave, ser=self.ser)

    def close(self):
        if not self.owns_port:
            return                      # the port belongs to the main connection
        try:
            self.ser.close()
        except Exception:
            pass

    def broadcast_write(self, addr, val):
        """Modbus broadcast write (slave 0): every drive acts, nobody replies.
        Used only if you opt in; results are verified through the encoder."""
        frame = bytes([0]) + struct.pack(">BHH", 6, addr, val & 0xFFFF)
        frame += crc16(frame)
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        time.sleep(0.05)
        self.ser.reset_input_buffer()   # discard any reply a drive might send

    def _xfer(self, pdu, expected, retries=2):
        frame = bytes([self.slave]) + pdu
        frame += crc16(frame)
        for _ in range(retries + 1):
            self.ser.reset_input_buffer()
            self.ser.write(frame)
            buf = self.ser.read(expected)
            if len(buf) >= 5 and buf[0] == self.slave and buf[-2:] == crc16(buf[:-2]):
                if buf[1] & 0x80:
                    self.last_exception = buf[2]
                    return None
                return buf
            time.sleep(0.02)
        return None

    def rd(self, addr, n=1):
        """Read n holding registers -> list of ints, or None."""
        buf = self._xfer(struct.pack(">BHH", 3, addr, n), 5 + 2 * n)
        if buf is None or buf[1] != 3 or len(buf) != 5 + 2 * n:
            return None
        return [struct.unpack(">H", buf[3 + 2 * i:5 + 2 * i])[0] for i in range(n)]

    def rd1(self, addr):
        v = self.rd(addr, 1)
        return v[0] if v else None

    def wr(self, addr, val):
        """Write one register -> True if the drive echoed it."""
        buf = self._xfer(struct.pack(">BHH", 6, addr, val & 0xFFFF), 8)
        return buf is not None and buf[1] == 6

    def wr_verify(self, addr, val, quiet=False):
        ok = self.wr(addr, val)
        back = self.rd1(addr)
        good = ok and back == (val & 0xFFFF)
        if not quiet:
            print(f"  reg {addr:>3} = {val:>6}  readback {back}  {'OK' if good else 'MISMATCH'}")
        return good

    def snapshot(self):
        """Regs 0..16 (alarm, status, speed, bus, error) plus reg 48 (encoder).
        The encoder is read on its own so it can never be lost by a block-read
        problem; if that read fails, the last good value is reused and
        enc_ok is set to False (the display then shows ENC READ FAIL)."""
        blk = self.rd(0, 17)
        if blk is None:
            return None
        enc = self.rd1(48)
        enc_ok = enc is not None
        if enc_ok:
            self.last_enc = enc
        return {
            "alarm": blk[0], "status": blk[1],
            "speed": s16(blk[10]), "bus": blk[11] / 100.0,
            "pos": s32(blk[8], blk[9]), "trk": s32(blk[12], blk[13]),
            "enc": self.last_enc, "enc_ok": enc_ok,
        }


class EncTracker:
    """Unwraps the single-turn count (reg 48) into a signed running total."""
    def __init__(self, resolution):
        self.res = max(1, resolution)
        self.last = None
        self.total = 0
        self.seen_max = 0

    def update(self, raw):
        self.seen_max = max(self.seen_max, raw)
        if self.last is not None:
            d = raw - self.last
            if d > self.res / 2:
                d -= self.res
            elif d < -self.res / 2:
                d += self.res
            self.total += d
        self.last = raw
        return self.total

    def degrees(self, counts=None):
        c = self.total if counts is None else counts
        return c / self.res * 360.0


class Config:
    def __init__(self):
        self.mode = 0                      # 0 = open-loop, 1 = servo
        self.speed = DEFAULT_SPEED_RPM
        self.accel = DEFAULT_ACCEL
        self.current = DEFAULT_CURRENT_MA
        self.enc_res = 16384
        self.ppr = 4000
        self.bus_start = 24.0


# ----------------------------------------------------------------------------
# Connect
# ----------------------------------------------------------------------------
def scan_ids(port, baud):
    print("Scanning slave IDs 1-31 (about 10 s)...")
    found = []
    for sid in range(1, 32):
        d = Drive(port, baud, sid)
        d.ser.timeout = 0.12
        v = d._xfer(struct.pack(">BHH", 3, 0, 1), 7, retries=0)
        d.close()
        if v is not None:
            found.append(sid)
            print(f"  reply from slave ID {sid}")
    if not found:
        print("  no drives answered. Check wiring (A/B), baud DIP switches, power.")
    return found


def connect():
    print("\n=== Connection setup ===")
    port = ask("COM port", DEFAULT_PORT)
    baud = ask_int("Baud rate (must match DIP SW6/SW7)", DEFAULT_BAUD, 1200, 921600)
    while True:
        slave = ask_int("Slave ID (1-31, set by DIP SW1-5)", 1, 1, 247)
        drive = Drive(port, baud, slave)
        info = drive.rd(0, 2)
        if info is not None:
            print(f"Connected: slave {slave} on {port} @ {baud}.")
            return drive
        print(f"No reply from slave {slave}.")
        drive.close()
        choice = ask("(r)etry a different ID, (s)can IDs 1-31, (q)uit", "r").lower()
        if choice.startswith("q"):
            sys.exit(0)
        if choice.startswith("s"):
            ids = scan_ids(port, baud)
            if ids:
                print(f"Use one of: {ids}")


# ----------------------------------------------------------------------------
# Actions
# ----------------------------------------------------------------------------
def read_cfg_from_drive(d, cfg):
    res = d.rd1(40)
    ppr = d.rd1(111)
    if res and res >= 256:
        cfg.enc_res = res
    if ppr and ppr >= 200:
        cfg.ppr = ppr
    s = d.snapshot()
    if s:
        cfg.bus_start = s["bus"]


def show_info(d, cfg):
    s = d.snapshot()
    if not s:
        print("No reply from drive.")
        return
    print("\n--- Device info ---")
    print(f"  Slave ID (DIP)    : {d.slave}")
    print(f"  Drive ID / version: {d.rd1(93)} / {d.rd1(94)}")
    print(f"  Bus voltage       : {s['bus']:.2f} V")
    print(f"  Alarm  reg0 = 0x{s['alarm']:04X} -> {decode_alarm(s['alarm'])}")
    print(f"  Status reg1 = 0x{s['status']:04X} -> {decode_status(s['status'])}")
    print(f"  Run mode (reg 22) : {d.rd1(22)}   (0 open-loop, 1 servo)")
    print(f"  Identified R / L  : {d.rd1(31)} mOhm / {d.rd1(32)} mH")
    print(f"  Encoder resolution: {cfg.enc_res} counts/rev, now at {s['enc']}")
    print("  Notes: RDY should be set; STALL can stay set on this unit even when it works.")


def configure(d, cfg):
    print("\n--- Configure ---")
    print("This writes the manual's recommended parameters and verifies each.")
    print("Motion settings (speed, accel, current, mode) come from menu 3.")
    if ask("Continue? (y/n)", "y").lower() != "y":
        return
    bad = 0
    for reg, val, desc in CONFIG_PARAMS:
        ok = d.wr_verify(reg, val, quiet=True)
        if not ok:
            bad += 1
            print(f"  reg {reg:>3} -> {val} MISMATCH ({desc}), read {d.rd1(reg)}")
    print(f"Wrote {len(CONFIG_PARAMS)} parameters, {bad} mismatches.")
    apply_runtime(d, cfg)
    read_cfg_from_drive(d, cfg)
    if ask("Save to flash now? (y/n)", "y").lower() == "y":
        save_flash(d)
        print(">>> POWER-CYCLE the drive now (24 V off ~5 s). At power-up the shaft")
        print(">>> should lock firmly and the drive re-identifies the motor.")


def save_flash(d):
    print("Saving to flash (reg 90 = 1):", "OK" if d.wr(90, 1) else "no echo")


def settings(d, cfg):
    while True:
        print("\n--- Settings ---")
        print(f"  1 run mode : {cfg.mode} ({'servo' if cfg.mode else 'open-loop'})")
        print(f"  2 speed    : {cfg.speed} rpm")
        print(f"  3 accel/decel: {cfg.accel} r/s^2")
        print(f"  4 current  : {cfg.current} mA")
        print("  b back")
        c = ask("Choose", "b").lower()
        if c == "1":
            cfg.mode = ask_int("Run mode: 0 = open-loop, 1 = servo", cfg.mode, 0, 1)
        elif c == "2":
            v = ask_int(f"Speed in rpm (1-{SPEED_MAX})", cfg.speed, 1, SPEED_MAX)
            if v > SPEED_CONFIRM_ABOVE and ask(
                    "Above 100 rpm without a braking resistor can trip over-voltage. "
                    "Continue? (y/n)", "n").lower() != "y":
                continue
            cfg.speed = v
        elif c == "3":
            cfg.accel = ask_int("Accel/decel in r/s^2 (10-1000, 10 = gentlest)",
                                cfg.accel, 10, 1000)
        elif c == "4":
            v = ask_int("Current in mA (0-6000)", cfg.current, 0, 6000)
            if v > 3000 and ask("High current heats the motor. Continue? (y/n)",
                                "n").lower() != "y":
                continue
            cfg.current = v
        else:
            break
    apply_runtime(d, cfg)


def apply_runtime(d, cfg):
    d.wr(18, 6)
    time.sleep(0.3)
    d.wr(22, cfg.mode)
    time.sleep(0.3)
    d.wr(17, 0)          # internal pulse source
    d.wr(20, 0)          # communication control
    d.wr(78, 0)          # incremental moves
    d.wr(110, cfg.current)
    for reg in (70, 71, 75, 76):     # position & jog accel/decel
        d.wr(reg, cfg.accel)
    d.wr(72, cfg.speed)
    d.wr(77, cfg.speed)


def safety_reason(s, cfg):
    if s["alarm"] != 0:
        return f"alarm 0x{s['alarm']:04X} ({decode_alarm(s['alarm'])})"
    if s["bus"] > cfg.bus_start + BUS_RISE_LIMIT_V:
        return f"bus voltage rose to {s['bus']:.2f} V"
    return None


HEADER = (f"{'t(s)':>6} {'bus V':>6} {'cmd rpm':>8} {'enc raw':>8} {'enc_total':>10} "
          f"{'deg':>10} {'revs':>8} {'enc_rpm':>8} {'trk_err':>8}  status")


def line(t, s, tr, rpm):
    return (f"{t:6.2f} {s['bus']:6.2f} {s['speed']:8d} {s['enc']:8d} {tr.total:10d} "
            f"{tr.degrees():10.2f} {tr.total / tr.res:8.3f} {rpm:8.1f} {s['trk']:8d}  "
            f"{decode_status(s['status'])}"
            f"{'' if s.get('enc_ok', True) else '  [ENC READ FAIL]'}")


def wait_until_stopped(d, cfg, tr, t0, timeout=20.0):
    """Poll until MOV clears, printing encoder lines."""
    t_end = time.time() + timeout
    last_t, last_total = time.time(), tr.total
    while time.time() < t_end:
        time.sleep(0.15)
        s = d.snapshot()
        if not s:
            continue
        tr.update(s["enc"])
        now = time.time()
        rpm = (tr.total - last_total) / tr.res * 60.0 / max(now - last_t, 1e-3)
        last_t, last_total = now, tr.total
        print(line(now - t0, s, tr, rpm))
        if not (s["status"] & (1 << 3)):
            return True
    print("  (motor still reports MOV after timeout)")
    return False


def summary(tr, start_total, elapsed, direction=None):
    delta = tr.total - start_total
    deg = tr.degrees(delta)
    print("\n--- Encoder summary ---")
    print(f"  Encoder moved: {delta} counts = {deg:.2f} deg = {deg / 360:.3f} rev")
    if elapsed > 0:
        print(f"  Average encoder speed over {elapsed:.1f} s: "
              f"{delta / tr.res * 60.0 / elapsed:.1f} rpm")
    if tr.seen_max >= tr.res:
        print(f"  WARNING: encoder count reached {tr.seen_max}, above the configured "
              f"resolution {tr.res}. Check reg 40.")
    if abs(delta) < 50:
        print("  No encoder movement detected -> shaft did not turn (or encoder is not "
              "connected).")
    elif direction == "f" and delta < 0 or direction == "r" and delta > 0:
        print("  Encoder counted OPPOSITE to the command. If you want them to agree,\n"
              "  flip SW8 (and power-cycle) or write reg 47 = 1 - change only one.")


def jog(d, cfg, direction):
    apply_runtime(d, cfg)
    secs = ask_int("Max jog duration in seconds", 5, 1, 120)
    if msvcrt:
        print("Press any key to stop early.")
    else:
        print("Press Ctrl+C to stop early.")
    s = d.snapshot()
    if not s:
        print("No reply from drive.")
        return
    tr = EncTracker(cfg.enc_res)
    tr.update(s["enc"])
    start_total = tr.total
    print(f"\nJog {'FORWARD' if direction == 'f' else 'REVERSE'} at {cfg.speed} rpm, "
          f"accel {cfg.accel} r/s^2, mode {'servo' if cfg.mode else 'open-loop'}")
    print(HEADER)
    d.wr(18, 3 if direction == "f" else 4)
    t0 = time.time()
    last_t, last_total = t0, tr.total
    reason = "time limit reached"
    warned = False
    try:
        while time.time() - t0 < secs:
            if key_pressed():
                reason = "key pressed"
                break
            time.sleep(0.1)
            s = d.snapshot()
            if not s:
                print("  (no reply)")
                continue
            tr.update(s["enc"])
            now = time.time()
            rpm = (tr.total - last_total) / tr.res * 60.0 / max(now - last_t, 1e-3)
            last_t, last_total = now, tr.total
            print(line(now - t0, s, tr, rpm))
            if (not warned and now - t0 > 2.0
                    and abs(tr.total - start_total) < 20):
                warned = True
                print("  WARNING: encoder counts are not changing although a jog is "
                      "commanded.\n  The shaft is probably not turning (or the encoder "
                      "is not connected).")
            why = safety_reason(s, cfg)
            if why:
                reason = "SAFETY ABORT: " + why
                break
    except KeyboardInterrupt:
        reason = "Ctrl+C"
    finally:
        d.wr(18, 6)
    print(f"Stopping ({reason})...")
    wait_until_stopped(d, cfg, tr, t0)
    summary(tr, start_total, time.time() - t0, direction)


def move_angle(d, cfg):
    apply_runtime(d, cfg)
    print("\nIncremental move. Positive = forward, negative = reverse.")
    unit = ask("Enter move in (d)egrees or (p)ulses", "d").lower()
    if unit.startswith("p"):
        pulses = int(ask_float("Pulses (signed)", 400))
    else:
        deg = ask_float("Degrees (signed, e.g. 90 or -45)", 90)
        pulses = int(round(deg / 360.0 * cfg.ppr))
    if pulses == 0:
        print("Nothing to do.")
        return
    cmd_deg = pulses / cfg.ppr * 360.0
    s = d.snapshot()
    if not s:
        print("No reply from drive.")
        return
    tr = EncTracker(cfg.enc_res)
    tr.update(s["enc"])
    start_total = tr.total
    dist = abs(pulses) & 0xFFFFFFFF
    d.wr(73, dist & 0xFFFF)               # low 16 bits of distance
    d.wr(74, (dist >> 16) & 0xFFFF)       # high 16 bits
    print(f"Commanding {pulses} pulses = {cmd_deg:.2f} deg at {cfg.speed} rpm.")
    print(HEADER)
    t0 = time.time()
    d.wr(18, 1 if pulses > 0 else 2)      # 1 = forward fixed length, 2 = reverse
    time.sleep(0.2)
    try:
        wait_until_stopped(d, cfg, tr, t0, timeout=30)
    except KeyboardInterrupt:
        d.wr(18, 6)
        print("Interrupted - stop sent.")
    # settle & measure
    time.sleep(0.4)
    s = d.snapshot()
    if s:
        tr.update(s["enc"])
    actual = tr.degrees(tr.total - start_total)
    print("\n--- Move result ---")
    print(f"  Commanded: {cmd_deg:.2f} deg")
    print(f"  Encoder  : {actual:.2f} deg ({tr.total - start_total} counts)")
    print(f"  Error    : {actual - cmd_deg:.2f} deg")
    if abs(actual) < 1.0:
        print("  Shaft did not turn (or encoder not counting).")
    elif cfg.mode == 0:
        print("  Open-loop: small error is normal if the drive loses steps; "
              "large error suggests overload or too-high accel.")


def monitor(d, cfg):
    print("\nLive encoder monitor. Turn the shaft by hand to see counts change.")
    print("Press any key (terminal) or Ctrl+C to stop. Duration limit 300 s.")
    s = d.snapshot()
    if not s:
        print("No reply from drive.")
        return
    tr = EncTracker(cfg.enc_res)
    tr.update(s["enc"])
    print(HEADER)
    t0 = time.time()
    last_t, last_total = t0, tr.total
    try:
        while time.time() - t0 < 300:
            if key_pressed():
                break
            time.sleep(0.15)
            s = d.snapshot()
            if not s:
                continue
            tr.update(s["enc"])
            now = time.time()
            rpm = (tr.total - last_total) / tr.res * 60.0 / max(now - last_t, 1e-3)
            last_t, last_total = now, tr.total
            print(line(now - t0, s, tr, rpm))
    except KeyboardInterrupt:
        pass
    print("Monitor stopped.")


# ----------------------------------------------------------------------------
# Dual-drive mode: two drives on the same RS485 bus move together
# ----------------------------------------------------------------------------
DUAL_HEADER = (f"{'t(s)':>6} | {'A bus':>6} {'A raw':>6} {'A total':>9} {'A deg':>9} "
               f"{'A rpm':>7} {'A status':<22}| {'B bus':>6} {'B raw':>6} {'B total':>9} "
               f"{'B deg':>9} {'B rpm':>7} {'B status':<22}| {'A-B deg':>8}")


def short_status(v, enc_ok=True):
    """Compact flags for the dual table: ALM STALL RDY MOV ENA (+ENCFAIL)."""
    keep = [(1, "ALM"), (10, "STALL"), (5, "RDY"), (3, "MOV"), (0, "ENA")]
    text = " ".join(n for b, n in keep if v & (1 << b)) or "-"
    return text + ("" if enc_ok else " ENCFAIL")


def dual_poll(ctx, trs, last, t0, inv):
    """Read both drives once. Returns (snaps, printable_line)."""
    snaps, parts, prog = [], [], []
    for i, (dr, c) in enumerate(ctx):
        s = dr.snapshot()
        snaps.append(s)
        if s is None:
            parts.append(f"{'(no reply)':>62}")
            prog.append(0.0)
            continue
        trs[i].update(s["enc"])
        now = time.time()
        rpm = (trs[i].total - last[i][1]) / trs[i].res * 60.0 / max(now - last[i][0], 1e-3)
        last[i] = (now, trs[i].total)
        st = short_status(s["status"], s["enc_ok"])
        parts.append(f"{s['bus']:6.2f} {s['enc']:6d} {trs[i].total:9d} "
                     f"{trs[i].degrees():9.2f} {rpm:7.1f} {st:<22}")
        prog.append(trs[i].degrees() * (-1 if inv[i] else 1))
    diff = prog[0] - prog[1]
    return snaps, (f"{time.time() - t0:6.2f} | {parts[0]}| {parts[1]}| {diff:8.2f}")


def dual_jog(d, cfg):
    print("\n=== Dual-drive jog ===")
    id_a = ask_int("Slave ID of drive A", 1, 1, 247)
    id_b = ask_int("Slave ID of drive B", 2, 1, 247)
    if id_a == id_b:
        print("The two IDs must be different.")
        return
    ctx = []
    for sid in (id_a, id_b):
        dr = d.clone(sid)
        if dr.rd(0, 2) is None:
            print(f"No reply from slave {sid}. Check its DIP switches, power and "
                  "RS485 wiring (both drives must share the same A/B pair).")
            return
        c = Config()
        c.mode, c.speed, c.accel, c.current = cfg.mode, cfg.speed, cfg.accel, cfg.current
        read_cfg_from_drive(dr, c)
        ctx.append((dr, c))
        print(f"  slave {sid}: OK (encoder {c.enc_res} counts/rev, bus {c.bus_start:.2f} V)")

    inv = [
        ask(f"Invert direction of drive {n} in software? (y/n)", "n").lower() == "y"
        for n in ("A", "B")
    ]
    secs = ask_int("Max duration in seconds", 5, 1, 120)
    use_bc = False
    if not any(inv):
        use_bc = ask("Try a broadcast start (both drives start at the exact same "
                     "instant; untested on this firmware) (y/n)", "n").lower() == "y"

    print(f"\nApplying settings to both drives: {cfg.speed} rpm, accel {cfg.accel} "
          f"r/s^2, mode {'servo' if cfg.mode else 'open-loop'} ...")
    for dr, c in ctx:
        apply_runtime(dr, c)

    trs, last = [], []
    for dr, c in ctx:
        s = dr.snapshot()
        if not s:
            print(f"Lost contact with slave {dr.slave}.")
            return
        tr = EncTracker(c.enc_res)
        tr.update(s["enc"])
        trs.append(tr)
        last.append((time.time(), 0))
    start_totals = [t.total for t in trs]

    print(f"\nDrive A = slave {id_a}{' (inverted)' if inv[0] else ''}   "
          f"Drive B = slave {id_b}{' (inverted)' if inv[1] else ''}")
    print("Both run POSITIVE velocity (command 3 = forward) unless inverted.")
    print("Press any key to stop early." if msvcrt else "Press Ctrl+C to stop early.")
    print(DUAL_HEADER)

    cmds = [4 if inv[0] else 3, 4 if inv[1] else 3]
    skew_ms = 0.0
    if use_bc:
        ctx[0][0].broadcast_write(18, 3)
        method = "broadcast (simultaneous)"
    else:
        t1 = time.perf_counter()
        ctx[0][0].wr(18, cmds[0])
        ctx[1][0].wr(18, cmds[1])
        skew_ms = (time.perf_counter() - t1) * 1000.0   # time between the two starts
        method = f"back-to-back writes (start offset about {skew_ms:.1f} ms)"
    print(f"Start method: {method}")

    t0 = time.time()
    last = [(t0, 0), (t0, 0)]
    for i in range(2):
        last[i] = (t0, trs[i].total)
    reason = "time limit reached"
    try:
        while time.time() - t0 < secs:
            if key_pressed():
                reason = "key pressed"
                break
            time.sleep(0.08)
            snaps, text = dual_poll(ctx, trs, last, t0, inv)
            print(text)
            for (dr, c), s in zip(ctx, snaps):
                if s:
                    why = safety_reason(s, c)
                    if why:
                        reason = f"SAFETY ABORT on slave {dr.slave}: {why}"
                        raise StopIteration
    except StopIteration:
        pass
    except KeyboardInterrupt:
        reason = "Ctrl+C"
    finally:
        for dr, _ in ctx:               # stop BOTH, as fast as possible
            dr.wr(18, 6)
    print(f"Stopping both ({reason})...")

    t_end = time.time() + 20
    while time.time() < t_end:
        time.sleep(0.15)
        snaps, text = dual_poll(ctx, trs, last, t0, inv)
        print(text)
        if all(s and not (s["status"] & (1 << 3)) for s in snaps):
            break

    print("\n=== Dual-drive summary ===")
    elapsed = time.time() - t0
    for (dr, c), tr, st0, iv, tag in zip(ctx, trs, start_totals, inv, ("A", "B")):
        print(f"\nDrive {tag} (slave {dr.slave}):")
        summary(tr, st0, elapsed, "r" if iv else "f")
    prog = [t.degrees(t.total - s0) * (-1 if iv else 1)
            for t, s0, iv in zip(trs, start_totals, inv)]
    print(f"\nSynchronization: A moved {prog[0]:.2f} deg, B moved {prog[1]:.2f} deg "
          f"(difference {prog[0] - prog[1]:.2f} deg).")
    if prog[0] * prog[1] < 0 or (min(abs(p) for p in prog) < 1.0):
        print("  One drive moved opposite to the other or did not move. If they spin\n"
              "  in opposite physical directions, flip SW8 on one drive (power-cycle)\n"
              "  or answer y to the software invert question for that drive.")
    print("  'Positive velocity' = forward command. Whether that looks clockwise depends\n"
          "  on SW8 and from which end you view the shaft; check it by eye once.")


def stop(d):
    print("Stop (reg 18 = 6):", "OK" if d.wr(18, 6) else "no echo")


def dump(d):
    print("\n--- Register dump ---")
    for reg, name in DUMP_REGS:
        v = d.rd1(reg)
        extra = ""
        if reg == 0 and v is not None:
            extra = f"  [{decode_alarm(v)}]"
        elif reg == 1 and v is not None:
            extra = f"  [{decode_status(v)}]"
        print(f"  reg {reg:>3} {name:<28} {v}{extra}")


MENU = """
================ IT86 TOOL ================
 1  Device info / status
 2  Configure (write recommended parameters)
 3  Settings (mode, speed, accel, current)
 4  Jog forward        5  Jog reverse
 6  Move by angle / pulses
 7  Live encoder monitor
 8  STOP motor
 9  Register dump
 d  DUAL-drive jog (two slave IDs together)
 s  Save to flash
 h  Help (full guide)
 q  Quit
===========================================
"""


# ----------------------------------------------------------------------------
# Instructions shown automatically when a menu option is selected
# ----------------------------------------------------------------------------
INSTRUCTIONS = {
    "1": """
INSTRUCTIONS - Device info / status
  What it does : reads the drive ID, firmware, bus voltage, alarm bits and
                 status bits, and the identified motor resistance/inductance.
  What to check: * Alarm should read "no alarms".
                 * Status should include ENA (enabled) and RDY (ready).
                 * Bus voltage should match your supply (about 24 V).
                 * STALL may stay set on this unit even when it works.
                 * Identified R/L should be measured values (not 1000 / 5);
                   if they are, run Configure, save, and power-cycle.
""",
    "2": """
INSTRUCTIONS - Configure
  What it does : writes the manual's recommended parameters (about 40
                 registers), reads each one back to verify, then offers to
                 save to flash.
  Before you start:
                 * Motor must be stationary and the supply on (24-50 V).
                 * Pole pairs (39) and out-of-tolerance (45) are not written.
  After it finishes:
                 * Answer y to save, then POWER-CYCLE the drive (24 V off
                   about 5 seconds). The shaft should lock firmly at power-up
                   and the drive re-measures the motor.
                 * Then run option 1 to confirm R/L are measured and RDY is set.
""",
    "3": """
INSTRUCTIONS - Settings
  What it does : sets run mode, speed, acceleration and current. They are
                 sent to the drive right before each jog or move.
  Run mode     : 0 = open-loop (start here), 1 = servo / closed-loop.
  Speed        : very low by default (20 rpm). With no braking resistor keep
                 it at or below 100 rpm.
  Accel/decel  : 10 r/s^2 is the manual minimum and the gentlest.
  Current      : 1500 mA is the default; above 3000 mA the motor heats up.
  Tip          : change one thing at a time and test with a short jog.
""",
    "4": """
INSTRUCTIONS - Jog FORWARD
  What it does : spins the motor continuously forward at the Settings speed
                 and prints live encoder data until the time limit, a key
                 press (terminal) or Ctrl+C.
  Before you start:
                 * Keep hands, hair and clothing away from the shaft.
                 * Start with open-loop mode, 20 rpm, a short duration.
  Watch for    : * enc_total increasing (counts rising) = shaft is turning.
                 * enc_rpm close to the commanded speed.
                 * The summary at the end shows the total encoder movement
                   and warns if the encoder counted the opposite way.
  To stop      : any key, Ctrl+C, or cut the 24 V supply for an emergency.
""",
    "5": """
INSTRUCTIONS - Jog REVERSE
  What it does : same as forward jog but in the opposite direction.
  Before you start:
                 * Keep hands clear of the shaft; start slow and short.
  Watch for    : * enc_total DEcreasing = shaft turning in reverse.
                 * If the encoder counts the same way as in forward jog,
                   the direction or encoder wiring is wrong (see option h).
  To stop      : any key, Ctrl+C, or cut the 24 V supply.
""",
    "6": """
INSTRUCTIONS - Move by angle / pulses
  What it does : turns the shaft by a fixed amount and then compares the
                 commanded angle with the angle measured by the encoder.
  How to use   : choose degrees or pulses, then enter a signed value.
                 Positive = forward, negative = reverse.
                 Example: 90 = quarter turn, 360 = one full turn.
                 With 4000 pulses/rev, 400 pulses = 36 degrees.
  Tip          : try 90 degrees first. A small error is normal in open-loop;
                 a large error means lost steps (lower speed/accel).
  To stop      : Ctrl+C sends a decelerating stop.
""",
    "7": """
INSTRUCTIONS - Live encoder monitor
  What it does : streams encoder, speed, bus voltage and status without
                 commanding any motion.
  How to use   : turn the shaft by hand (the drive holds position, so use
                 gentle force) and watch enc_raw and enc_total change.
  Reading it   : * enc_raw   = single-turn count, wraps at the resolution.
                 * enc_total = running total (signed) since you started.
                 * deg/revs  = total converted to degrees / revolutions.
  To stop      : press any key (terminal) or Ctrl+C. Limit 300 seconds.
""",
    "8": """
INSTRUCTIONS - STOP
  What it does : sends a decelerating stop (reg 18 = 6). The motor slows
                 using the deceleration setting and holds position.
  Note         : for a true emergency, cut the 24 V supply.
""",
    "9": """
INSTRUCTIONS - Register dump
  What it does : prints the key registers with names and decoded alarm and
                 status bits. Use it to compare against the manual, or paste
                 the output when asking for support.
""",
    "d": """
INSTRUCTIONS - Dual-drive jog (two drives together)
  What it does : runs two drives on the same RS485 bus at the same time at
                 positive velocity and shows both encoders live in one table
                 (A = first slave ID, B = second), plus the angle difference.
  Before you start:
                 * Both drives need DIFFERENT slave IDs (DIP SW1-5), the
                   same baud rate, and share the same RS485 A/B pair.
                 * Both should be configured and saved (option 2) and
                   powered with the same supply. Power-cycle after DIP changes.
                 * Keep hands clear of BOTH shafts.
  Settings used: run mode, speed, accel and current from option 3.
  Start method : by default two back-to-back Modbus writes (about 2-5 ms
                 apart; the offset is printed). You may opt in to a broadcast
                 write so both start at the same instant - untested on this
                 firmware, so check the encoder values afterwards.
  Direction    : positive velocity = forward command. The physical direction
                 depends on SW8 and on how each motor is mounted. If the two
                 shafts turn opposite ways, flip SW8 on one drive or answer y
                 to the software invert question for that drive.
  Safety       : an alarm or a bus voltage rise on EITHER drive stops BOTH.
  To stop      : any key (terminal), Ctrl+C, or cut the 24 V supply.
""",
    "s": """
INSTRUCTIONS - Save to flash
  What it does : writes 1 to register 90 so the current parameters are kept
                 across power cycles. Settings like speed and accel from
                 option 3 are re-applied each time you jog, so they do not
                 need saving. Power-cycle afterwards for lock-shaft or
                 identification changes to take effect.
""",
}


def show_instructions(choice):
    """Print the instructions for a menu choice; return False if the user cancels."""
    text = INSTRUCTIONS.get(choice)
    if not text:
        return True
    print(text)
    try:
        ans = input("Press Enter to continue, or type c to cancel: ").strip().lower()
    except KeyboardInterrupt:
        return False
    return ans != "c"


def main():
    print(__doc__.split("-----", 1)[0])  # short banner
    d = connect()
    cfg = Config()
    read_cfg_from_drive(d, cfg)
    print(f"Encoder resolution {cfg.enc_res}, pulses/rev {cfg.ppr}, "
          f"bus {cfg.bus_start:.2f} V.")
    print(f"Safe defaults: {cfg.speed} rpm, accel {cfg.accel} r/s^2, mode open-loop.")
    try:
        while True:
            print(MENU)
            c = ask("Choose", "1").lower()
            if c in INSTRUCTIONS and not show_instructions(c):
                print("Cancelled.")
                continue
            if c == "1":
                show_info(d, cfg)
            elif c == "2":
                configure(d, cfg)
            elif c == "3":
                settings(d, cfg)
            elif c == "4":
                jog(d, cfg, "f")
            elif c == "5":
                jog(d, cfg, "r")
            elif c == "6":
                move_angle(d, cfg)
            elif c == "7":
                monitor(d, cfg)
            elif c == "8":
                stop(d)
            elif c == "9":
                dump(d)
            elif c == "d":
                dual_jog(d, cfg)
            elif c == "s":
                save_flash(d)
            elif c == "h":
                print(__doc__)
            elif c == "q":
                break
    except serial.SerialException as e:
        print("Serial error:", e)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        try:
            d.wr(18, 6)
        except Exception:
            pass
        d.close()
        print("Motor stop sent, port closed.")


if __name__ == "__main__":
    main()
