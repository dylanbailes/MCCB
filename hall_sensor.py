#!/usr/bin/env python3
"""
hall_sensor.py - Raspberry Pi 5 interface for STM32G431 dual DRV5055A1 firmware

Usage:
    python3 hall_sensor.py            # interactive mode
    python3 hall_sensor.py --auto     # auto-report mode (Ctrl+C to stop)
    python3 hall_sensor.py --stream   # raw ADC stream mode (Ctrl+C to stop)
    python3 hall_sensor.py --log      # auto-report + save to CSV file

Serial port: /dev/serial0 (maps to ttyAMA0 on Pi 5)
Baud:        9600

Prerequisites:
    sudo apt install python3-serial
    /boot/firmware/config.txt must have:
        enable_uart=1
        dtoverlay=uart0-pi5
    Login shell over serial must be disabled (raspi-config → Interface Options → Serial)
"""

import serial
import sys
import time
import argparse
import csv
import os
import re
from datetime import datetime


# ── Config ──────────────────────────────────────────────────────────────────────
PORT       = "/dev/serial0"
BAUD       = 9600
TIMEOUT    = 2.0      # seconds to wait for a response line
LOG_DIR    = os.path.expanduser("~/hall_logs")


# ── Serial open ─────────────────────────────────────────────────────────────────

def open_port(port=PORT, baud=BAUD):
    try:
        ser = serial.Serial(
            port        = port,
            baudrate    = baud,
            bytesize    = serial.EIGHTBITS,
            parity      = serial.PARITY_NONE,
            stopbits    = serial.STOPBITS_ONE,
            timeout     = TIMEOUT,
        )
        return ser
    except serial.SerialException as e:
        print(f"[ERROR] Could not open {port}: {e}")
        print("Check:")
        print("  1. /boot/firmware/config.txt has enable_uart=1 and dtoverlay=uart0-pi5")
        print("  2. raspi-config → Interface Options → Serial Port → login shell OFF")
        print("  3. STM32 TX→Pi GPIO15 (pin 10), RX→Pi GPIO14 (pin 8), GND→GND")
        print("  4. ST-Link 3.3V is NOT connected at the same time as Pi 3.3V")
        sys.exit(1)


# ── Read a full report block ─────────────────────────────────────────────────────
#
# Firmware output for 'R':
#   === INSTANT ===
#   S1(PA1): RAW=2051  mV=1652  mT=0.15
#   S2(PA6): RAW=2048  mV=1650  mT=0.00
#   === RMS (512 samples, AC only) ===
#   S1(PA1): RMS_RAW=2063  RMS_mV=1661  AC_RMS_mT=0.73
#   S2(PA6): RMS_RAW=2055  RMS_mV=1655  AC_RMS_mT=0.42
#   --------

def read_report_lines(ser, timeout=3.0):
    """Read lines until we see the '--------' terminator or timeout."""
    lines = []
    deadline = time.time() + timeout
    while time.time() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode("ascii", errors="replace").strip()
        if line:
            lines.append(line)
        if line == "--------":
            break
    return lines


def parse_report(lines):
    """
    Parse a report block into a dict. Returns None if parsing fails.
    Keys: s1_raw, s1_mv, s1_mt, s2_raw, s2_mv, s2_mt,
          s1_rms_raw, s1_rms_mv, s1_ac_rms_mt,
          s2_rms_raw, s2_rms_mv, s2_ac_rms_mt,
          s1_sat, s2_sat
    """
    data = {}

    def extract(line, key):
        """Pull 'KEY=value' from a line, return as string or None."""
        m = re.search(rf'{key}=(-?\d+(?:\.\d+)?)', line)
        return m.group(1) if m else None

    for line in lines:
        if line.startswith("S1(PA1):") and "RAW=" in line and "RMS" not in line:
            data["s1_raw"] = extract(line, "RAW")
            data["s1_mv"]  = extract(line, "mV")
            data["s1_mt"]  = extract(line, "mT")
            data["s1_sat"] = "SAT" in line

        elif line.startswith("S2(PA6):") and "RAW=" in line and "RMS" not in line:
            data["s2_raw"] = extract(line, "RAW")
            data["s2_mv"]  = extract(line, "mV")
            data["s2_mt"]  = extract(line, "mT")
            data["s2_sat"] = "SAT" in line

        elif line.startswith("S1(PA1):") and "RMS_RAW=" in line:
            data["s1_rms_raw"]    = extract(line, "RMS_RAW")
            data["s1_rms_mv"]     = extract(line, "RMS_mV")
            data["s1_ac_rms_mt"]  = extract(line, "AC_RMS_mT")

        elif line.startswith("S2(PA6):") and "RMS_RAW=" in line:
            data["s2_rms_raw"]    = extract(line, "RMS_RAW")
            data["s2_rms_mv"]     = extract(line, "RMS_mV")
            data["s2_ac_rms_mt"]  = extract(line, "AC_RMS_mT")

    # Require at least the instant values to be present
    if not all(k in data for k in ("s1_raw", "s2_raw")):
        return None
    return data


def print_report(data, timestamp=None):
    """Pretty-print a parsed report dict."""
    ts = timestamp or datetime.now().strftime("%H:%M:%S.%f")[:-3]
    sat1 = " ⚠ SATURATED" if data.get("s1_sat") else ""
    sat2 = " ⚠ SATURATED" if data.get("s2_sat") else ""

    print(f"\n[{ts}]")
    print(f"  ── Instantaneous ──────────────────────────────")
    print(f"  S1 (PA1):  RAW={data.get('s1_raw','?'):>4}  "
          f"mV={data.get('s1_mv','?'):>4}  "
          f"mT={data.get('s1_mt','?'):>7}{sat1}")
    print(f"  S2 (PA6):  RAW={data.get('s2_raw','?'):>4}  "
          f"mV={data.get('s2_mv','?'):>4}  "
          f"mT={data.get('s2_mt','?'):>7}{sat2}")
    print(f"  ── AC RMS (512 samples) ───────────────────────")
    print(f"  S1 (PA1):  RMS_RAW={data.get('s1_rms_raw','?'):>4}  "
          f"RMS_mV={data.get('s1_rms_mv','?'):>4}  "
          f"AC_RMS_mT={data.get('s1_ac_rms_mt','?'):>7}")
    print(f"  S2 (PA6):  RMS_RAW={data.get('s2_rms_raw','?'):>4}  "
          f"RMS_mV={data.get('s2_rms_mv','?'):>4}  "
          f"AC_RMS_mT={data.get('s2_ac_rms_mt','?'):>7}")


# ── CSV logging ──────────────────────────────────────────────────────────────────

CSV_FIELDS = [
    "timestamp",
    "s1_raw", "s1_mv", "s1_mt", "s1_sat",
    "s2_raw", "s2_mv", "s2_mt", "s2_sat",
    "s1_rms_raw", "s1_rms_mv", "s1_ac_rms_mt",
    "s2_rms_raw", "s2_rms_mv", "s2_ac_rms_mt",
]

def open_csv():
    os.makedirs(LOG_DIR, exist_ok=True)
    fname = os.path.join(LOG_DIR, datetime.now().strftime("hall_%Y%m%d_%H%M%S.csv"))
    f = open(fname, "w", newline="")
    writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
    writer.writeheader()
    print(f"[LOG] Writing to {fname}")
    return f, writer, fname

def write_csv_row(writer, data):
    row = {"timestamp": datetime.now().isoformat()} | {k: data.get(k, "") for k in CSV_FIELDS[1:]}
    writer.writerow(row)


# ── Modes ────────────────────────────────────────────────────────────────────────

def flush_boot_messages(ser):
    """Drain any pending bytes (boot banner, leftover data) before sending commands."""
    ser.reset_input_buffer()
    time.sleep(0.1)
    ser.reset_input_buffer()


def mode_single(ser):
    """Send 'R', print one report."""
    flush_boot_messages(ser)
    ser.write(b'R')
    lines = read_report_lines(ser)
    if not lines:
        print("[ERROR] No response — is the STM32 powered and running? Check wiring.")
        return
    # Print raw lines for debugging
    print("[RAW]", " | ".join(lines))
    data = parse_report(lines)
    if data:
        print_report(data)
    else:
        print("[WARN] Could not parse report. Raw output above.")


def mode_auto(ser, log=False):
    """Toggle auto-report on the STM32 ('A') and print/log continuously."""
    flush_boot_messages(ser)
    csv_file = csv_writer = csv_name = None
    if log:
        csv_file, csv_writer, csv_name = open_csv()

    print("Auto-report mode. Ctrl+C to stop.")
    ser.write(b'A')   # enable auto on firmware side

    try:
        while True:
            lines = read_report_lines(ser, timeout=5.0)
            if not lines:
                print("[WARN] Timeout waiting for data...")
                continue
            data = parse_report(lines)
            if data:
                print_report(data)
                if csv_writer:
                    write_csv_row(csv_writer, data)
                    csv_file.flush()
            else:
                # Print raw if we can't parse (e.g. boot banner)
                for l in lines:
                    print(f"  [raw] {l}")
    except KeyboardInterrupt:
        print("\nStopping...")
        ser.write(b'A')   # toggle auto off
        time.sleep(0.2)
    finally:
        if csv_file:
            csv_file.close()
            print(f"[LOG] Saved to {csv_name}")


def mode_stream(ser):
    """
    Send 'S' to start raw ADC streaming.
    Prints S1/S2 raw counts as fast as they arrive.
    Any keypress (Enter in terminal) stops it — or Ctrl+C.
    """
    flush_boot_messages(ser)
    print("Raw ADC stream mode. Press Ctrl+C to stop.")
    print(f"{'#':>6}  {'S1_RAW':>7}  {'S2_RAW':>7}  {'S1_mT':>8}  {'S2_mT':>8}")
    print("-" * 50)

    ser.write(b'S')
    count = 0

    # DRV5055A1 conversion constants (mirror firmware)
    MIDSCALE   = 2048
    SENS_NUM   = 330000
    SENS_DEN   = 245760

    def raw_to_mt(raw):
        delta = raw - MIDSCALE
        return (delta * SENS_NUM) / SENS_DEN / 100.0

    try:
        while True:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("ascii", errors="replace").strip()
            if line.startswith("STREAM"):
                print(f"  {line}")
                if "STOP" in line:
                    break
                continue
            # Parse "S1=NNN S2=NNN"
            m = re.match(r'S1=(\d+)\s+S2=(\d+)', line)
            if m:
                s1, s2 = int(m.group(1)), int(m.group(2))
                mt1, mt2 = raw_to_mt(s1), raw_to_mt(s2)
                count += 1
                print(f"{count:>6}  {s1:>7}  {s2:>7}  {mt1:>+8.2f}  {mt2:>+8.2f}")
    except KeyboardInterrupt:
        print("\nSending stop...")
        ser.write(b'X')   # any byte stops stream
        time.sleep(0.3)
        ser.reset_input_buffer()


def mode_interactive(ser):
    """Simple interactive terminal for manual commands."""
    flush_boot_messages(ser)
    print(f"Connected to {PORT} at {BAUD} baud.")
    print("Commands:  R = report   A = toggle auto   S = stream raw   Q = quit")
    print("-" * 60)

    # Print any pending boot banner
    ser.timeout = 0.5
    boot = ser.read(512)
    if boot:
        print(boot.decode("ascii", errors="replace"), end="")
    ser.timeout = TIMEOUT

    while True:
        try:
            cmd = input("\n> ").strip().upper()
        except (EOFError, KeyboardInterrupt):
            print("\nExiting.")
            break

        if not cmd:
            continue
        if cmd == "Q":
            break
        if cmd not in ("R", "A", "S"):
            print("Unknown command. Use R, A, S, or Q.")
            continue

        ser.write(cmd.encode())

        if cmd == "R":
            lines = read_report_lines(ser)
            data  = parse_report(lines)
            if data:
                print_report(data)
            else:
                for l in lines:
                    print(f"  {l}")
        elif cmd == "A":
            # Read and print lines until user presses Enter
            print("Auto mode (press Enter to stop)...")
            import threading, select
            stop = threading.Event()
            def reader():
                while not stop.is_set():
                    lines = read_report_lines(ser, timeout=1.0)
                    data  = parse_report(lines)
                    if data:
                        print_report(data)
            t = threading.Thread(target=reader, daemon=True)
            t.start()
            input()
            stop.set()
            ser.write(b'A')   # toggle off
            t.join(timeout=2)
        elif cmd == "S":
            mode_stream(ser)


# ── Entry point ──────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Raspberry Pi 5 interface for STM32 dual DRV5055A1 Hall sensor firmware"
    )
    parser.add_argument("--auto",   action="store_true", help="Auto-report mode (continuous)")
    parser.add_argument("--stream", action="store_true", help="Raw ADC stream mode")
    parser.add_argument("--log",    action="store_true", help="Auto-report + save CSV to ~/hall_logs/")
    parser.add_argument("--port",   default=PORT,        help=f"Serial port (default: {PORT})")
    parser.add_argument("--baud",   default=BAUD, type=int, help=f"Baud rate (default: {BAUD})")
    args = parser.parse_args()

    ser = open_port(args.port, args.baud)
    print(f"Opened {args.port} at {args.baud} baud.")

    try:
        if args.stream:
            mode_stream(ser)
        elif args.auto or args.log:
            mode_auto(ser, log=args.log)
        else:
            mode_interactive(ser)
    finally:
        ser.close()


if __name__ == "__main__":
    main()