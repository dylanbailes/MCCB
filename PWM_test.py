#!/usr/bin/env python3
"""
pwm_test.py - DRV8874 PWM test interface for STM32G431
Raspberry Pi 5 | /dev/serial0 | 9600 baud

Commands:
  F = Forward (IN1 PWM, IN2 low)
  V = Reverse (IN1 low, IN2 PWM)
  B = Brake   (IN1 high, IN2 high)
  C = Coast   (IN1 low, IN2 low)
  + = Duty +10%
  - = Duty -10%
  P = Print current state
  Q = Quit
"""

import serial
import sys
import time
from datetime import datetime


PORT = "/dev/serial0"
BAUD = 9600


def log(msg):
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    print(f"[{ts}] {msg}")


def open_port():
    try:
        ser = serial.Serial(
            port     = PORT,
            baudrate = BAUD,
            timeout  = 1.0,
        )
        log(f"Opened {PORT} at {BAUD} baud")
        return ser
    except serial.SerialException as e:
        print(f"ERROR: Could not open {PORT}: {e}")
        sys.exit(1)


def read_response(ser):
    """Read lines until no more arrive within 500ms."""
    lines = []
    ser.timeout = 0.5
    while True:
        raw = ser.readline()
        if not raw:
            break
        line = raw.decode("ascii", errors="replace").strip()
        if line:
            lines.append(line)
    return lines


def send_command(ser, cmd):
    log(f"Sending: {cmd!r}")
    ser.write(cmd.encode())
    time.sleep(0.05)
    lines = read_response(ser)
    for line in lines:
        log(f"  STM32: {line}")
    if not lines:
        log("  STM32: (no response)")


def print_help():
    print()
    print("  F = Forward    V = Reverse    B = Brake    C = Coast")
    print("  + = Duty +10%  - = Duty -10%  P = State    Q = Quit")
    print()


def main():
    ser = open_port()

    # Drain boot banner
    time.sleep(0.2)
    boot = ser.read(512)
    if boot:
        for line in boot.decode("ascii", errors="replace").splitlines():
            if line.strip():
                log(f"  STM32: {line.strip()}")

    print()
    print("DRV8874 PWM Test")
    print("─" * 40)
    print("Measure PA8 (IN1) and PA9 (IN2) with multimeter:")
    print("  Coast   → PA8=0V,     PA9=0V")
    print("  Forward → PA8=~1.65V, PA9=0V    (at 50% duty)")
    print("  Reverse → PA8=0V,     PA9=~1.65V")
    print("  Brake   → PA8=3.3V,   PA9=3.3V")
    print("─" * 40)
    print_help()

    VALID = {"F", "V", "B", "C", "+", "-", "P", "Q"}

    while True:
        try:
            cmd = input("> ").strip().upper()
        except (EOFError, KeyboardInterrupt):
            print()
            log("Exiting — sending Coast before quit")
            send_command(ser, "C")
            break

        if not cmd:
            continue

        if cmd not in VALID:
            print(f"  Unknown command '{cmd}'")
            print_help()
            continue

        if cmd == "Q":
            log("Exiting — sending Coast before quit")
            send_command(ser, "C")
            break

        send_command(ser, cmd)

    ser.close()
    log("Port closed")


if __name__ == "__main__":
    main()