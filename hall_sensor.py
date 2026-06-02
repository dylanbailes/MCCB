#!/usr/bin/env python3
"""
hall_sensor.py - Raspberry Pi 5 interface for STM32G431 unified firmware
(hall sensing + INA240A1 current sense + VBUS monitor + DRV8874 PWM)

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


# ── Config ───────────────────────────────────────────────────────────────────────
PORT    = "/dev/serial0"
BAUD    = 9600
TIMEOUT = 2.0
LOG_DIR = os.path.expanduser("~/hall_logs")

# DRV5055A1 conversion constants — mirror firmware
_MIDSCALE = 2048
_SENS_NUM  = 330000
_SENS_DEN  = 245760

def _raw_to_mt(raw):
    return ((raw - _MIDSCALE) * _SENS_NUM) / _SENS_DEN / 100.0


# ── Serial open ──────────────────────────────────────────────────────────────────

def open_port(port=PORT, baud=BAUD):
    try:
        ser = serial.Serial(
            port     = port,
            baudrate = baud,
            bytesize = serial.EIGHTBITS,
            parity   = serial.PARITY_NONE,
            stopbits = serial.STOPBITS_ONE,
            timeout  = TIMEOUT,
        )
        return ser
    except serial.SerialException as e:
        print(f"[ERROR] Could not open {port}: {e}")
        print("Check:")
        print("  1. /boot/firmware/config.txt has enable_uart=1 and dtoverlay=uart0-pi5")
        print("  2. raspi-config → Interface Options → Serial Port → login shell OFF")
        print("  3. STM32 TX→Pi GPIO15 (pin 10), RX→Pi GPIO14 (pin 8), GND→GND")
        sys.exit(1)


# ── Report parsing ───────────────────────────────────────────────────────────────
#
# Firmware output for 'R':
#   === POWER ===
#   VBUS=24.02V  VBUS_RAW=2705
#   ISENSE=0.00A  ISENSE_RAW=2048
#   === INSTANT ===
#   S1(PA1): RAW=2029  mT=0.00
#   S2(PA6): RAW=2018  mT=0.00
#   === RMS (512 samples, AC, zeroed) ===
#   S1(PA1): RMS_RAW=2031  AC_RMS_mT=0.01
#   S2(PA6): RMS_RAW=2019  AC_RMS_mT=0.01
#   PWM: COAST  duty=0%  CCR=0
#   DRV: nSLEEP=1(AWAKE) nFAULT=1(OK)
#   --------

def read_report_lines(ser, timeout=3.0):
    """Read lines until '--------' terminator or timeout."""
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
    Parse a report block into a dict. Returns None if required fields missing.

    Keys: vbus_v, vbus_raw, isense_a, isense_raw, isense_warn,
          s1_raw, s1_mt, s1_sat, s2_raw, s2_mt, s2_sat,
          s1_rms_raw, s1_ac_rms_mt, s2_rms_raw, s2_ac_rms_mt,
          pwm_mode, pwm_duty_pct, pwm_ccr,
          drv_nsleep, drv_nsleep_state, drv_nfault, drv_nfault_state
    """
    data = {}

    def extract(line, key):
        m = re.search(rf'{key}=(-?\d+(?:\.\d+)?)', line)
        return m.group(1) if m else None

    for line in lines:
        if line.startswith("VBUS="):
            data["vbus_v"]   = extract(line, "VBUS")
            data["vbus_raw"] = extract(line, "VBUS_RAW")

        elif line.startswith("ISENSE="):
            data["isense_a"]    = extract(line, "ISENSE")
            data["isense_raw"]  = extract(line, "ISENSE_RAW")
            data["isense_warn"] = "***" in line

        elif line.startswith("S1(PA1):") and "RAW=" in line and "RMS" not in line:
            data["s1_raw"] = extract(line, "RAW")
            data["s1_mt"]  = extract(line, "mT")
            data["s1_sat"] = "SAT" in line

        elif line.startswith("S2(PA6):") and "RAW=" in line and "RMS" not in line:
            data["s2_raw"] = extract(line, "RAW")
            data["s2_mt"]  = extract(line, "mT")
            data["s2_sat"] = "SAT" in line

        elif line.startswith("S1(PA1):") and "RMS_RAW=" in line:
            data["s1_rms_raw"]   = extract(line, "RMS_RAW")
            data["s1_ac_rms_mt"] = extract(line, "AC_RMS_mT")

        elif line.startswith("S2(PA6):") and "RMS_RAW=" in line:
            data["s2_rms_raw"]   = extract(line, "RMS_RAW")
            data["s2_ac_rms_mt"] = extract(line, "AC_RMS_mT")

        elif line.startswith("PWM:"):
            # "PWM: FORWARD  duty=50%  CCR=499"
            m = re.match(r'PWM:\s+(\w+)\s+duty=(\d+)%\s+CCR=(\d+)', line)
            if m:
                data["pwm_mode"]     = m.group(1)
                data["pwm_duty_pct"] = m.group(2)
                data["pwm_ccr"]      = m.group(3)

        elif line.startswith("DRV:"):
            # "DRV: nSLEEP=1(AWAKE) nFAULT=1(OK)"
            m = re.match(r'DRV:\s+nSLEEP=(\d+)\((\w+)\)\s+nFAULT=(\d+)\((\w+)\)', line)
            if m:
                data["drv_nsleep"]       = m.group(1)
                data["drv_nsleep_state"] = m.group(2)
                data["drv_nfault"]       = m.group(3)
                data["drv_nfault_state"] = m.group(4)

    if not all(k in data for k in ("s1_raw", "s2_raw")):
        return None
    return data


def print_report(data, timestamp=None):
    """Pretty-print a parsed report dict."""
    ts   = timestamp or datetime.now().strftime("%H:%M:%S.%f")[:-3]
    sat1 = " ⚠ SATURATED" if data.get("s1_sat") else ""
    sat2 = " ⚠ SATURATED" if data.get("s2_sat") else ""
    vwarn = "  ⚠ CHECK" if not data.get("vbus_v") or float(data.get("vbus_v") or 0) < 1.0 else ""
    iwarn = "  ⚠ >100mA" if data.get("isense_warn") else ""

    print(f"\n[{ts}]")
    print(f"  ── Power ──────────────────────────────────────")
    print(f"  VBUS:      {data.get('vbus_v','?'):>7} V   RAW={data.get('vbus_raw','?'):>4}{vwarn}")
    print(f"  Current:   {data.get('isense_a','?'):>7} A   RAW={data.get('isense_raw','?'):>4}{iwarn}")
    print(f"  ── Instantaneous ──────────────────────────────")
    print(f"  S1 (PA1):  RAW={data.get('s1_raw','?'):>4}  mT={data.get('s1_mt','?'):>7}{sat1}")
    print(f"  S2 (PA6):  RAW={data.get('s2_raw','?'):>4}  mT={data.get('s2_mt','?'):>7}{sat2}")
    print(f"  ── AC RMS (512 samples) ───────────────────────")
    print(f"  S1 (PA1):  RMS_RAW={data.get('s1_rms_raw','?'):>4}  AC_RMS_mT={data.get('s1_ac_rms_mt','?'):>7}")
    print(f"  S2 (PA6):  RMS_RAW={data.get('s2_rms_raw','?'):>4}  AC_RMS_mT={data.get('s2_ac_rms_mt','?'):>7}")
    print(f"  ── PWM ────────────────────────────────────────")
    print(f"  Mode:      {data.get('pwm_mode','?'):<10}  duty={data.get('pwm_duty_pct','?'):>3}%  CCR={data.get('pwm_ccr','?')}")
    
    print(f"  ── DRV Status ─────────────────────────────────")
    nsleep_val = data.get('drv_nsleep', '?')
    nsleep_state = data.get('drv_nsleep_state', '?')
    nfault_val = data.get('drv_nfault', '?')
    nfault_state = data.get('drv_nfault_state', '?')
    
    swarn = " ⚠ SLEEPING" if nsleep_state == "SLEEP" else ""
    fwarn = " ⚠ FAULT!" if nfault_state == "FAULT" else ""
    
    print(f"  nSLEEP:    {nsleep_val} ({nsleep_state}){swarn}")
    print(f"  nFAULT:    {nfault_val} ({nfault_state}){fwarn}")


# ── CSV logging ──────────────────────────────────────────────────────────────────

CSV_FIELDS = [
    "timestamp",
    "vbus_v", "vbus_raw",
    "isense_a", "isense_raw",
    "s1_raw", "s1_mt", "s1_sat",
    "s2_raw", "s2_mt", "s2_sat",
    "s1_rms_raw", "s1_ac_rms_mt",
    "s2_rms_raw", "s2_ac_rms_mt",
    "pwm_mode", "pwm_duty_pct", "pwm_ccr",
    "drv_nsleep", "drv_nsleep_state", "drv_nfault", "drv_nfault_state",
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


# ── Helpers ──────────────────────────────────────────────────────────────────────

def flush_boot_messages(ser):
    """Drain pending bytes (boot banner, leftover data) before sending commands."""
    ser.reset_input_buffer()
    time.sleep(0.1)
    ser.reset_input_buffer()

# All valid single-character commands accepted by the firmware
VALID_CMDS = {"R", "A", "S", "Z", "F", "V", "B", "C", "+", "-", "P", "N", "D"}


# ── Modes ────────────────────────────────────────────────────────────────────────

def mode_single(ser):
    """Send 'R', print one report."""
    flush_boot_messages(ser)
    ser.write(b'R')
    lines = read_report_lines(ser)
    if not lines:
        print("[ERROR] No response — is the STM32 powered and running? Check wiring.")
        return
    data = parse_report(lines)
    if data:
        print_report(data)
    else:
        print("[WARN] Could not parse report.")
        for l in lines:
            print(f"  [raw] {l}")


def mode_auto(ser, log=False):
    """Toggle auto-report on the STM32 ('A') and print/log continuously."""
    flush_boot_messages(ser)
    csv_file = csv_writer = csv_name = None
    if log:
        csv_file, csv_writer, csv_name = open_csv()

    print("Auto-report mode. Ctrl+C to stop.")
    ser.write(b'A')

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


def mode_stream(ser, already_started=False):
    """
    Send 'S' to start raw ADC streaming.
    Firmware sends space-separated bare numbers: VBUS_RAW S1_RAW S2_RAW ISENSE_RAW
    Ctrl+C (or any keypress from firmware side) to stop.
    """
    print("Raw ADC stream. Ctrl+C to stop.")
    print(f"{'#':>6}  {'VBUS_RAW':>8}  {'S1_RAW':>6}  {'S2_RAW':>6}  {'ISENSE_RAW':>10}  {'S1_mT':>8}  {'S2_mT':>8}")
    print("-" * 70)

    if not already_started:
        flush_boot_messages(ser)
        ser.write(b'S')

    time.sleep(0.05)
    ser.timeout = 1.0
    count = 0

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
            # Firmware streams: "VBUS_RAW S1_RAW S2_RAW ISENSE_RAW" (bare numbers)
            parts = line.split()
            if len(parts) == 4 and all(p.isdigit() for p in parts):
                vbus_r, s1_r, s2_r, isense_r = (int(p) for p in parts)
                mt1 = _raw_to_mt(s1_r)
                mt2 = _raw_to_mt(s2_r)
                count += 1
                print(f"{count:>6}  {vbus_r:>8}  {s1_r:>6}  {s2_r:>6}  {isense_r:>10}  {mt1:>+8.2f}  {mt2:>+8.2f}")
    except KeyboardInterrupt:
        print("\nSending stop...")
        ser.write(b'X')   # any byte stops firmware stream
        time.sleep(0.3)
        ser.reset_input_buffer()
    finally:
        ser.timeout = TIMEOUT


def mode_interactive(ser):
    """Simple interactive terminal for manual commands."""
    flush_boot_messages(ser)
    print(f"Connected to {PORT} at {BAUD} baud.")
    print("Commands:")
    print("  R=report   A=auto     S=stream   Z=recalibrate")
    print("  F=forward  V=reverse  B=brake    C=coast")
    print("  +=duty+10% -=duty-10% P=pwm state  D=drv status  N=nSleep toggle  Q=quit")
    print("-" * 60)

    # Drain any boot banner
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
        if cmd not in VALID_CMDS:
            print(f"Unknown command '{cmd}'. Valid: {', '.join(sorted(VALID_CMDS))}, Q")
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
            print("Auto mode (press Enter to stop)...")
            import threading
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
            mode_stream(ser, already_started=True)

        elif cmd == "Z":
            print("Recalibrating — coast is applied automatically.")
            print("Ensure no external field sources are active...")
            lines = read_report_lines(ser, timeout=10.0)
            for l in lines:
                print(f"  {l}")

        elif cmd in ("F", "V", "B", "C", "+", "-", "P", "N", "D"):
            # Single-line response from firmware (e.g. "FORWARD\r\nPWM: FORWARD  duty=50%  CCR=499\r\n")
            # Read until 2s timeout collects the short ack
            time.sleep(0.1)
            resp = ser.read(256)
            if resp:
                print(resp.decode("ascii", errors="replace"), end="")


# ── Entry point ──────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Raspberry Pi 5 interface for STM32G431 unified Helmholtz coil firmware"
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