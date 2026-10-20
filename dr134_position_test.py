#!/usr/bin/env python3
"""Fixed-pulse position test for an IT86AM85 via a USR-DR134.

Starts one selected drive, then compares registers 8-9 before, during, and after motion.
Use only with the robot secured and the tested wheel safely raised.
"""

import socket
import struct
import time

# ============================================================
# CONFIGURATION
# ============================================================

HOST = "192.168.169.5"
PORT = 1502

SLAVE_ID = 2              # Test motor 2 first; then change to 1
TEST_PULSES = 4000        # Nominally one revolution
DIRECTION = "forward"     # "forward" or "reverse"

SOCKET_TIMEOUT = 5.0
POLL_INTERVAL = 0.25
MOVE_TIMEOUT = 15.0

# Registers
REG_POSITION = 8
REG_PULSES_PER_REV = 111
REG_DISTANCE_LOW = 73
REG_DISTANCE_HIGH = 74
REG_COMMAND = 18

# Commands from your previous working script
CMD_FORWARD = 1
CMD_REVERSE = 2
CMD_STOP = 6


# ============================================================
# MODBUS RTU CRC
# ============================================================

def crc16(data):
    crc = 0xFFFF

    for byte in data:
        crc ^= byte

        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1

    return crc & 0xFFFF


def make_frame(payload):
    data = bytes([SLAVE_ID]) + payload
    crc = crc16(data)

    return data + bytes([
        crc & 0xFF,
        (crc >> 8) & 0xFF
    ])


def receive_exact(sock, count):
    data = b""

    while len(data) < count:
        chunk = sock.recv(count - len(data))

        if not chunk:
            raise ConnectionError(
                "Connection closed while receiving response"
            )

        data += chunk

    return data


# ============================================================
# IT86AM85 MODBUS CLIENT
# ============================================================

class IT86Drive:

    def __init__(self):
        print(f"Connecting to {HOST}:{PORT} ...")

        self.sock = socket.create_connection(
            (HOST, PORT),
            timeout=SOCKET_TIMEOUT
        )

        self.sock.settimeout(SOCKET_TIMEOUT)

        print("TCP connection successful.")

    def request(self, payload, expected_length, description):
        request_frame = make_frame(payload)

        print(f"\n[REQUEST] {description}")
        print("TX:", request_frame.hex(" "))

        self.sock.sendall(request_frame)

        print("Request sent; waiting for response...")

        response = receive_exact(
            self.sock,
            expected_length
        )

        print("RX:", response.hex(" "))

        received_crc = (
            response[-2] |
            (response[-1] << 8)
        )

        calculated_crc = crc16(response[:-2])

        if received_crc != calculated_crc:
            raise RuntimeError(
                f"CRC mismatch during {description}"
            )

        if response[0] != SLAVE_ID:
            raise RuntimeError(
                f"Unexpected slave ID: {response.hex()}"
            )

        if response[1] & 0x80:
            raise RuntimeError(
                f"Modbus exception during {description}: "
                f"{response.hex()}"
            )

        return response

    def read_registers(self, address, count):
        payload = struct.pack(
            ">BHH",
            3,
            address,
            count
        )

        response = self.request(
            payload,
            5 + count * 2,
            f"Read registers {address}-{address + count - 1}"
        )

        byte_count = response[2]

        if byte_count != count * 2:
            raise RuntimeError(
                f"Unexpected byte count: {response.hex()}"
            )

        return [
            int.from_bytes(
                response[3 + 2 * i:5 + 2 * i],
                "big"
            )
            for i in range(count)
        ]

    def write_register(self, address, value):
        payload = struct.pack(
            ">BHH",
            6,
            address,
            value & 0xFFFF
        )

        response = self.request(
            payload,
            8,
            f"Write register {address} = {value}"
        )

        # Function 06 normally echoes address and value.
        if response[1:6] != payload:
            raise RuntimeError(
                "Write response did not match request: "
                + response.hex()
            )

    def position(self):
        # Registers 8-9: low word followed by high word
        low, high = self.read_registers(
            REG_POSITION,
            2
        )

        value = (high << 16) | low

        if value & 0x80000000:
            value -= 0x100000000

        return value

    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass


# ============================================================
# TEST
# ============================================================

def main():

    if SLAVE_ID not in (1, 2):
        raise ValueError("SLAVE_ID must be 1 or 2")

    if DIRECTION not in ("forward", "reverse"):
        raise ValueError(
            'DIRECTION must be "forward" or "reverse"'
        )

    drive = None
    command_sent = False
    initial_position = None
    pulses_per_rev = None

    print("=" * 60)
    print("IT86AM85 POSITION TEST")
    print("=" * 60)
    print(f"Slave ID: {SLAVE_ID}")
    print(f"Requested movement: {TEST_PULSES} pulses")
    print(f"Direction: {DIRECTION}")
    print()
    print("SAFETY:")
    print("- Secure the robot and raise the tested wheel.")
    print("- Keep clear of rotating components.")
    print("- Keep the emergency stop accessible.")
    print("- Do not run another program controlling this drive.")
    print()

    input("Press Enter to connect... ")

    try:
        drive = IT86Drive()

        print("\nReading initial position...")
        initial_position = drive.position()

        print("\nReading pulses-per-revolution setting...")
        pulses_per_rev = drive.read_registers(
            REG_PULSES_PER_REV,
            1
        )[0]

        print("\nINITIAL VALUES")
        print("Position:", initial_position)
        print("Register 111:", pulses_per_rev)

        input(
            "\nPress Enter to configure and start the move, "
            "or Ctrl+C to cancel... "
        )

        # Configure the requested distance.
        # These are the same registers used by the previous test.
        drive.write_register(
            REG_DISTANCE_LOW,
            TEST_PULSES & 0xFFFF
        )

        drive.write_register(
            REG_DISTANCE_HIGH,
            (TEST_PULSES >> 16) & 0xFFFF
        )

        # Check the position immediately before movement.
        before_move = drive.position()

        print("\nPosition immediately before movement:",
              before_move)

        command = (
            CMD_FORWARD
            if DIRECTION == "forward"
            else CMD_REVERSE
        )

        # A write may reach the drive even if its response times out.
        # Do not automatically retry this command.
        # Mark the command as potentially sent before writing: if the reply
        # times out, the drive may still have received the movement command.
        command_sent = True
        try:
            drive.write_register(
                REG_COMMAND,
                command
            )

        except Exception:
            print(
                "\nWARNING: movement command response failed. "
                "The command may still have reached the drive."
            )
            raise

        start_time = time.monotonic()
        previous_position = before_move

        print("\nMonitoring position...")

        while True:
            time.sleep(POLL_INTERVAL)

            current_position = drive.position()
            elapsed = time.monotonic() - start_time

            total_change = current_position - before_move
            step_change = current_position - previous_position

            print(
                f"{elapsed:6.2f}s | "
                f"position={current_position:12d} | "
                f"delta={total_change:+d} | "
                f"step={step_change:+d}"
            )

            previous_position = current_position

            if abs(total_change) >= TEST_PULSES:
                print("\nTarget position change reached.")
                break

            if elapsed >= MOVE_TIMEOUT:
                print("\nMonitoring timeout reached.")
                break

    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    except Exception as exc:
        print(
            f"\nERROR: {type(exc).__name__}: {exc}"
        )

    finally:
        if drive is not None:

            # Request stop if movement was commanded.
            if command_sent:
                try:
                    print("\nSending stop command...")
                    drive.write_register(
                        REG_COMMAND,
                        CMD_STOP
                    )
                    print("Stop command acknowledged.")

                except Exception as exc:
                    print(
                        "WARNING: stop command failed:",
                        repr(exc)
                    )
                    print(
                        "Use the drive's documented stop or "
                        "emergency-stop procedure if motion continues."
                    )

            if initial_position is not None:
                try:
                    print("\nReading final position...")
                    final_position = drive.position()

                    print("\n" + "=" * 60)
                    print("TEST RESULTS")
                    print("=" * 60)
                    print("Slave ID:", SLAVE_ID)
                    print("Initial position:", initial_position)
                    print("Final position:", final_position)

                    net_change = (
                        final_position - initial_position
                    )

                    print("Net position change:", net_change)
                    print("Requested pulses:", TEST_PULSES)

                    if TEST_PULSES:
                        print(
                            "Measured change / requested pulses:",
                            f"{net_change / TEST_PULSES:.4f}"
                        )

                    if pulses_per_rev:
                        print(
                            "Nominal revolutions from register 111:",
                            f"{net_change / pulses_per_rev:.4f}"
                        )

                except Exception as exc:
                    print(
                        "Final position read failed:",
                        repr(exc)
                    )

            drive.close()
            print("\nConnection closed.")


if __name__ == "__main__":
    main()
