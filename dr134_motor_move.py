#!/usr/bin/env python3
"""
dr134_motor_move.py - move IT86 drives (slave IDs 1 and 2) by N degrees over a
USR-DR134 RS485->Ethernet converter (TCP, port 1502).

Uses exactly the same Modbus registers as your working it86_tool.py:
  reg 18 = 1/2   fixed-length move forward / reverse
  reg 73/74      move distance in pulses (low / high 16 bits)
  reg 111        pulses per revolution (4000 -> 180 deg = 2000 pulses)
  reg 48         single-turn encoder (unwrapped for angle reporting; validate on your firmware)
Standalone: no other files needed, only the Python standard library.

Run:   python dr134_motor_move.py                     (both drives, +180 deg)
       python dr134_motor_move.py --deg -180          (reverse)
       python dr134_motor_move.py --ids 1 --deg 90
       python dr134_motor_move.py --check             (only talk to drives, never move)
       python dr134_motor_move.py --framing tcp       (if the DR134 is in Modbus-TCP mode)
"""
import argparse
import socket
import struct
import sys
import time

MOV_BIT = 1 << 3


# ----------------------------------------------------------------------------
# Modbus helpers
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


class Link:
    """One TCP connection to the DR134, shared by all drives on the RS485 bus."""

    def __init__(self, host, port, framing="rtu", timeout=0.6):
        self.host, self.port, self.framing, self.timeout = host, port, framing, timeout
        self.sock = None
        self.tid = 0
        self.connect()

    def connect(self):
        self.close()
        self.sock = socket.create_connection((self.host, self.port), timeout=3)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

    def _drain(self):
        self.sock.settimeout(0)
        try:
            while self.sock.recv(4096):
                pass
        except (BlockingIOError, socket.timeout, OSError):
            pass

    def _recv(self, n):
        """Read up to n bytes, stop at n or when the timeout expires."""
        buf, end = b"", time.monotonic() + self.timeout
        while len(buf) < n and time.monotonic() < end:
            self.sock.settimeout(max(0.01, end - time.monotonic()))
            try:
                chunk = self.sock.recv(n - len(buf))
            except socket.timeout:
                break
            if not chunk:
                raise ConnectionError("DR134 closed the connection")
            buf += chunk
        return buf

    def xfer(self, unit, pdu, resp_pdu_len, retries=2):
        """Send one request, return the response PDU (bytes) or None."""
        for attempt in range(retries + 1):
            try:
                if self.sock is None:
                    self.connect()
                self._drain()
                if self.framing == "rtu":
                    frame = bytes([unit]) + pdu
                    self.sock.sendall(frame + crc16(frame))
                    buf = self._recv(1 + resp_pdu_len + 2)
                    ok = (len(buf) >= 5 and buf[0] == unit and buf[-2:] == crc16(buf[:-2]))
                    body = buf[1:-2] if ok else None
                else:
                    self.tid = (self.tid + 1) & 0xFFFF
                    self.sock.sendall(struct.pack(">HHHB", self.tid, 0, len(pdu) + 1, unit) + pdu)
                    buf = self._recv(7 + resp_pdu_len)
                    ok = len(buf) >= 9 and buf[6] == unit and buf[:2] == struct.pack(">H", self.tid)
                    body = buf[7:] if ok else None
                if body:
                    if body[0] & 0x80:           # Modbus exception: the drive IS alive
                        return None
                    return body
            except (OSError, ConnectionError):
                self.close()
            time.sleep(0.03)
        return None


class Drive:
    def __init__(self, link, unit):
        self.link, self.unit = link, unit

    def rd(self, addr, n=1):
        r = self.link.xfer(self.unit, struct.pack(">BHH", 3, addr, n), 2 + 2 * n)
        if not r or r[0] != 3 or r[1] != 2 * n:
            return None
        return [struct.unpack(">H", r[2 + 2 * i:4 + 2 * i])[0] for i in range(n)]

    def rd1(self, addr):
        v = self.rd(addr, 1)
        return v[0] if v else None

    def wr(self, addr, val):
        r = self.link.xfer(self.unit, struct.pack(">BHH", 6, addr, val & 0xFFFF), 5)
        return bool(r) and r[0] == 6


def decode_alarm(v):
    names = {8: "encoder fault", 7: "tracking error", 6: "phase loss", 5: "parameter error",
             4: "over-temp", 3: "under-voltage", 2: "over-voltage", 1: "over-current",
             0: "internal voltage"}
    return ", ".join(n for b, n in names.items() if v & (1 << b)) or "none"


def apply_runtime(d, mode, speed, accel, current):
    """Apply the runtime settings used by the interactive IT86 tool."""
    d.wr(18, 6)
    time.sleep(0.3)
    d.wr(22, mode)
    time.sleep(0.3)
    d.wr(17, 0)          # command source = internal pulse
    d.wr(20, 0)          # application = communication control
    d.wr(78, 0)          # incremental moves
    d.wr(110, current)
    for reg in (70, 71, 75, 76):
        d.wr(reg, accel)
    d.wr(72, speed)
    d.wr(77, speed)


class Enc:
    """Unwraps the single-turn encoder (reg 48) into a signed running total."""

    def __init__(self, res):
        self.res, self.last, self.total = max(1, res), None, 0

    def update(self, raw):
        if self.last is not None:
            dlt = raw - self.last
            if dlt > self.res / 2:
                dlt -= self.res
            elif dlt < -self.res / 2:
                dlt += self.res
            self.total += dlt
        self.last = raw

    def deg(self):
        return self.total / self.res * 360.0


# ----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--host", default="192.168.169.5")
    ap.add_argument("--port", type=int, default=1502)
    ap.add_argument("--ids", default="1,2", help="slave IDs, comma separated")
    ap.add_argument("--deg", type=float, default=180.0, help="signed angle in degrees (default: 180)")
    ap.add_argument("--speed", type=int, default=20, help="rpm (default 20, gentle)")
    ap.add_argument("--accel", type=int, default=10, help="r/s^2 (default 10, gentlest)")
    ap.add_argument("--current", type=int, default=1500, help="mA")
    ap.add_argument("--mode", type=int, default=0, choices=(0, 1), help="0 open-loop, 1 servo")
    ap.add_argument("--framing", default="rtu", choices=("rtu", "tcp"),
                    help="rtu = Modbus-RTU bytes inside TCP (transparent mode, default)")
    ap.add_argument("--check", action="store_true", help="only read the drives, do not move")
    ap.add_argument("--yes", action="store_true", help="skip the confirmation prompt")
    a = ap.parse_args()

    ids = [int(x) for x in a.ids.split(",")]
    print(f"Connecting to {a.host}:{a.port} ({a.framing} framing) ...")
    try:
        link = Link(a.host, a.port, a.framing)
    except OSError as e:
        sys.exit(f"Cannot connect: {e}")

    drives, ctx = [], {}
    try:
        # ---- 1. talk to every drive, never move if one is silent -------------
        for u in ids:
            d = Drive(link, u)
            regs = d.rd(0, 2)
            if regs is None:
                sys.exit(f"\nNo valid reply from slave {u}. Nothing was moved.\n"
                         f"Check: DR134 port {a.port} / serial 115200 8N1, RS485 wiring, drive "
                         f"power, DIP address; try --framing tcp.")
            res, ppr = d.rd1(40), d.rd1(111)
            res = res if res and res >= 256 else 16384
            ppr = ppr if ppr and ppr >= 200 else 4000
            bus = (d.rd1(11) or 0) / 100.0
            print(f"  slave {u}: OK  alarm=0x{regs[0]:04X} ({decode_alarm(regs[0])}) "
                  f"status=0x{regs[1]:04X}  bus={bus:.1f} V  enc={res} cnt/rev  ppr={ppr}")
            if regs[0]:
                sys.exit(f"Slave {u} has an alarm active. Clear it (power-cycle) first.")
            drives.append(d)
            ctx[u] = (res, ppr)

        if a.check:
            print("\nCheck passed: all drives answer. No motion commanded.")
            return

        pulses = {u: int(round(a.deg / 360.0 * ctx[u][1])) for u in ids}
        if any(p == 0 for p in pulses.values()):
            sys.exit("Angle is too small for the pulses-per-rev setting.")
        print(f"\nPlan: move slave(s) {ids} by {a.deg:+.1f} deg "
              f"({', '.join(f'{u}: {p:+d} pulses' for u, p in pulses.items())}) "
              f"at {a.speed} rpm, accel {a.accel} r/s^2, "
              f"{'servo' if a.mode else 'open-loop'} mode.")
        if not a.yes and input("Keep clear of the shafts. Press Enter to start (q to quit): ") \
                .strip().lower() == "q":
            return

        # ---- 2. configure and read start encoder ------------------------------
        encs = {}
        for d in drives:
            apply_runtime(d, a.mode, a.speed, a.accel, a.current)
            raw = d.rd1(48)
            if raw is None:
                sys.exit(f"Could not read encoder of slave {d.unit}.")
            encs[d.unit] = Enc(ctx[d.unit][0])
            encs[d.unit].update(raw)

        # ---- 3. load distances, then start both back-to-back ------------------
        for d in drives:
            dist = abs(pulses[d.unit]) & 0xFFFFFFFF
            ok = d.wr(73, dist & 0xFFFF) and d.wr(74, (dist >> 16) & 0xFFFF)
            if not ok:
                sys.exit(f"Slave {d.unit}: distance write not acknowledged. Aborting.")
        t0 = time.perf_counter()
        for d in drives:
            if not d.wr(18, 1 if pulses[d.unit] > 0 else 2):
                for x in drives:
                    x.wr(18, 6)
                sys.exit(f"Slave {d.unit}: start command not acknowledged. Stopped all.")
        print(f"Started (offset between drives about {(time.perf_counter() - t0) * 1000:.1f} ms)")
        print(f"{'t(s)':>6} " + " ".join(f"| slave {u}: {'deg':>8} status" for u in ids))

        # ---- 4. watch until every drive reports MOV cleared -------------------
        t_start = time.time()
        timeout = abs(a.deg) / 360.0 / max(a.speed, 1) * 60.0 * 2.5 + 10
        seen_move = {u: False for u in ids}
        done = False
        while time.time() - t_start < timeout and not done:
            time.sleep(0.15)
            parts, all_idle = [], True
            for d in drives:
                st, raw = d.rd1(1), d.rd1(48)
                if st is None or raw is None:
                    parts.append("(no reply)".rjust(24))
                    all_idle = False
                    continue
                encs[d.unit].update(raw)
                moving = bool(st & MOV_BIT)
                seen_move[d.unit] |= moving
                if moving or not seen_move[d.unit]:
                    all_idle = False
                al = d.rd1(0)
                if al:
                    for x in drives:
                        x.wr(18, 6)
                    sys.exit(f"\nALARM on slave {d.unit}: {decode_alarm(al)}. Stop sent to all.")
                parts.append(f"{encs[d.unit].deg():8.2f} {'MOV' if moving else 'idle'}".rjust(24))
            print(f"{time.time() - t_start:6.2f} | " + " | ".join(parts))
            done = all_idle
            if not any(seen_move.values()) and time.time() - t_start > 3:
                print("No MOV flag seen after 3 s - drives did not start moving.")
                break

        time.sleep(0.4)
        print("\n--- Result (measured by the encoders) ---")
        for d in drives:
            raw = d.rd1(48)
            if raw is not None:
                encs[d.unit].update(raw)
            cmd = pulses[d.unit] / ctx[d.unit][1] * 360.0
            meas = encs[d.unit].deg()
            print(f"  slave {d.unit}: commanded {cmd:+.2f} deg, measured {meas:+.2f} deg, "
                  f"error {meas - cmd:+.2f} deg")
    except KeyboardInterrupt:
        print("\nCtrl+C - sending stop to all drives.")
    finally:
        for d in drives:
            try:
                d.wr(18, 6)
            except Exception:
                pass
        link.close()


if __name__ == "__main__":
    main()
