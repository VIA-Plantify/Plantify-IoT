#!/usr/bin/env python3
"""
plantify_logger.py  -  Plantify-IoT data logger
================================================
Two modes, auto-detected:

  MODE 1 - TCP (button 3 held at boot)
    Arduino creates WiFi AP "PlantPot-Data".
    Connect your PC to that WiFi, then run this script.

  MODE 2 - Serial (normal boot, USB cable)
    Arduino prints sensor lines over USB serial.
    Live readings are shown in the terminal every second.
    CSV is written every 20 minutes.

USAGE
-----
  python plantify_logger.py              # auto mode
  python plantify_logger.py --mode tcp
  python plantify_logger.py --mode serial
  python plantify_logger.py --mode serial --serialport COM3
  python plantify_logger.py --out my_plants.csv

INSTALL DEPENDENCY (one time only)
-----
  pip install pyserial

CSV COLUMNS
-----------
  timestamp, temperature_c, humidity_pct, light_raw, soil_pct, distance_mm
"""

import argparse
import csv
import getpass
import json
import os
import re
import socket
import sys
import time
from datetime import datetime

# ── Try importing pyserial ────────────────────────────────────────────────────
try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

# ── Defaults ──────────────────────────────────────────────────────────────────
DEFAULT_HOST     = "192.168.4.1"
DEFAULT_PORT     = 9000
DEFAULT_BAUD     = 115200
DEFAULT_OUT      = "plantify_data.csv"
TCP_TIMEOUT_S    = 5
CSV_INTERVAL_S   = 20 * 60   # write to CSV every 20 minutes

CSV_FIELDS = [
    "timestamp",
    "temperature_c",
    "humidity_pct",
    "light_raw",
    "soil_pct",
    "distance_mm",
]

# ── Regex for serial line:  T:23.1C H:55.0% L:320 S:72 D:180 M:0 ────────────
SERIAL_RE = re.compile(
    r"T:(\d+)\.(\d+)C\s+"
    r"H:(\d+)\.(\d+)%\s+"
    r"L:(\d+)\s+"
    r"S:(\d+)\s+"
    r"D:(\d+)"
)


# ── Helpers ───────────────────────────────────────────────────────────────────

def log(msg):
    print(f"[{datetime.now().strftime('%H:%M:%S')}] {msg}")


# ── CSV ───────────────────────────────────────────────────────────────────────

def open_csv(path):
    is_new = not os.path.exists(path) or os.path.getsize(path) == 0
    fh = open(path, "a", newline="", encoding="utf-8")
    writer = csv.DictWriter(fh, fieldnames=CSV_FIELDS)
    if is_new:
        writer.writeheader()
        fh.flush()
        log(f"Created CSV: {path}")
    else:
        log(f"Appending to existing CSV: {path}")
    return fh, writer


def make_row(temp_c, hum_pct, light, soil, dist):
    return {
        "timestamp":     datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "temperature_c": round(temp_c, 1),
        "humidity_pct":  round(hum_pct, 1),
        "light_raw":     int(light),
        "soil_pct":      int(soil),
        "distance_mm":   int(dist),
    }


# ── Serial helpers ────────────────────────────────────────────────────────────

def find_arduino_port():
    if not SERIAL_AVAILABLE:
        return None
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None
    log(f"Found {len(ports)} serial port(s):")
    for p in ports:
        log(f"  {p.device}  -  {p.description}")
    keywords = ["arduino", "ch340", "ftdi", "usb serial", "uart", "mega"]
    for p in ports:
        if any(kw in p.description.lower() for kw in keywords):
            log(f"Auto-selected: {p.device} ({p.description})")
            return p.device
    log(f"No Arduino keyword match - trying first port: {ports[0].device}")
    return ports[0].device


def parse_serial_line(line):
    m = SERIAL_RE.search(line)
    if not m:
        return None
    temp  = float(f"{m.group(1)}.{m.group(2)}")
    hum   = float(f"{m.group(3)}.{m.group(4)}")
    light = int(m.group(5))
    soil  = int(m.group(6))
    dist  = int(m.group(7))
    return temp, hum, light, soil, dist


# ── Serial mode ───────────────────────────────────────────────────────────────

def run_serial(port, baud, writer, fh):
    rows          = 0
    last_csv_write = 0   # 0 = write on first reading immediately

    while True:
        ser = None
        while ser is None:
            target = port or find_arduino_port()
            if target is None:
                log("No serial port found. Is the Arduino plugged in via USB? Retrying in 3s ...")
                time.sleep(3)
                continue
            try:
                log(f"Opening serial port {target} at {baud} baud ...")
                ser = serial.Serial(target, baud, timeout=2)
                log(f"Serial connected on {target}!")
            except serial.SerialException as e:
                log(f"Could not open port: {e}. Retrying in 3s ...")
                time.sleep(3)

        log("Live readings shown every second. CSV saved every 20 minutes.")
        print("-" * 50)

        try:
            while True:
                raw = ser.readline()
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                # Print all Arduino output
                print(f"  [arduino] {line}")

                parsed = parse_serial_line(line)
                if parsed is None:
                    continue   # boot message or other non-sensor line

                row = make_row(*parsed)
                now = time.time()

                # Calculate time until next CSV save
                elapsed   = now - last_csv_write
                remaining = max(0, CSV_INTERVAL_S - elapsed)
                mins      = int(remaining // 60)
                secs      = int(remaining % 60)

                # Show live reading in terminal every 60 seconds
                if not hasattr(run_serial, '_last_print') or now - run_serial._last_print >= 60:
                    print(
                        f"  [live]    T={row['temperature_c']}C  "
                        f"H={row['humidity_pct']}%  "
                        f"Light={row['light_raw']}  "
                        f"Soil={row['soil_pct']}%  "
                        f"Dist={row['distance_mm']}mm  "
                        f"| next CSV save in {mins}m {secs:02d}s"
                    )
                    run_serial._last_print = now

                # Write to CSV only every 20 minutes
                if elapsed >= CSV_INTERVAL_S:
                    writer.writerow(row)
                    fh.flush()
                    rows += 1
                    last_csv_write = now
                    log(f"*** Saved row #{rows} to CSV at {row['timestamp']} ***")

        except (OSError, serial.SerialException) as e:
            log(f"Serial error: {e}. Reconnecting ...")
        finally:
            try:
                ser.close()
            except Exception:
                pass
        print("-" * 50)


# ── TCP mode ──────────────────────────────────────────────────────────────────

def try_tcp_connect(host, port):
    try:
        log(f"Trying TCP connection to {host}:{port} ...")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(TCP_TIMEOUT_S)
        sock.connect((host, port))
        sock.settimeout(None)
        log("TCP connected!")
        return sock
    except (ConnectionRefusedError, OSError, socket.timeout) as e:
        log(f"TCP failed: {e}")
        return None


def parse_json_line(line):
    try:
        data  = json.loads(line)
        temp  = float(data.get("temp",  0))
        hum   = float(data.get("hum",   0))
        light = int(data.get("light", 0))
        soil  = int(data.get("soil",  0))
        dist  = int(data.get("dist",  0))
        return temp, hum, light, soil, dist
    except (json.JSONDecodeError, ValueError, KeyError) as e:
        log(f"Bad JSON ({e}): {line!r}")
        return None


def run_tcp(host, port, writer, fh):
    rows          = 0
    buf           = ""
    last_csv_write = 0

    while True:
        sock = None
        while sock is None:
            sock = try_tcp_connect(host, port)
            if sock is None:
                log("Retrying in 3s ... (make sure Arduino is in button-3 mode and PC is on PlantPot-Data WiFi)")
                time.sleep(3)

        log("Live readings shown every second. CSV saved every 20 minutes.")
        try:
            while True:
                chunk = sock.recv(512)
                if not chunk:
                    log("Arduino closed the connection. Reconnecting ...")
                    break

                buf += chunk.decode("utf-8", errors="replace")

                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue

                    parsed = parse_json_line(line)
                    if parsed is None:
                        continue

                    row = make_row(*parsed)
                    now = time.time()

                    elapsed   = now - last_csv_write
                    remaining = max(0, CSV_INTERVAL_S - elapsed)
                    mins      = int(remaining // 60)
                    secs      = int(remaining % 60)

                    print(
                        f"  [live]  T={row['temperature_c']}C  "
                        f"H={row['humidity_pct']}%  "
                        f"Light={row['light_raw']}  "
                        f"Soil={row['soil_pct']}%  "
                        f"Dist={row['distance_mm']}mm  "
                        f"| next CSV save in {mins}m {secs:02d}s"
                    )

                    if elapsed >= CSV_INTERVAL_S:
                        writer.writerow(row)
                        fh.flush()
                        rows += 1
                        last_csv_write = now
                        log(f"*** Saved row #{rows} to CSV at {row['timestamp']} ***")

        except (OSError, ConnectionResetError) as e:
            log(f"TCP error: {e}. Reconnecting ...")
        finally:
            try:
                sock.close()
            except Exception:
                pass


# ── Auto mode ─────────────────────────────────────────────────────────────────

def run_auto(host, port, serial_port, baud, writer, fh):
    log("Auto mode: trying TCP first ...")
    sock = try_tcp_connect(host, port)
    if sock is not None:
        sock.close()
        log("TCP reachable - running in TCP mode.")
        log("Tip: to force serial mode use --mode serial")
        run_tcp(host, port, writer, fh)
    else:
        log("TCP not reachable - switching to serial (USB) mode.")
        log("Tip: to force TCP mode connect to PlantPot-Data WiFi and use --mode tcp")
        if not SERIAL_AVAILABLE:
            print("\n  ERROR: pyserial not installed. Run:  pip install pyserial\n")
            sys.exit(1)
        run_serial(serial_port, baud, writer, fh)


# ── Entry point ───────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description="Plantify-IoT data logger")
    p.add_argument("--mode", choices=["auto", "tcp", "serial"], default="auto",
                   help="Connection mode (default: auto)")
    p.add_argument("--host", default=DEFAULT_HOST,
                   help=f"Arduino IP for TCP mode (default: {DEFAULT_HOST})")
    p.add_argument("--port", type=int, default=DEFAULT_PORT,
                   help=f"TCP port (default: {DEFAULT_PORT})")
    p.add_argument("--serialport", default=None,
                   help="Serial port e.g. COM3 (default: auto-detect)")
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                   help=f"Serial baud rate (default: {DEFAULT_BAUD})")
    p.add_argument("--out", default=DEFAULT_OUT,
                   help=f"Output CSV file (default: {DEFAULT_OUT})")
    return p.parse_args()


def main():
    args = parse_args()

    print("=" * 60)
    print("  Plantify-IoT Logger")
    print("=" * 60)
    print(f"  Mode       : {args.mode}")
    print(f"  TCP target : {args.host}:{args.port}")
    print(f"  Serial     : {args.serialport or 'auto-detect'} @ {args.baud} baud")
    print(f"  Output     : {args.out}")
    print(f"  CSV interval: every 20 minutes")
    print("  Press Ctrl+C to stop.")
    print("=" * 60)
    print()

    if args.mode in ("serial", "auto") and not SERIAL_AVAILABLE:
        print("  WARNING: pyserial not installed - serial mode unavailable.")
        print("  Run:  pip install pyserial\n")
        if args.mode == "serial":
            sys.exit(1)

    fh, writer = open_csv(args.out)

    try:
        if args.mode == "tcp":
            run_tcp(args.host, args.port, writer, fh)
        elif args.mode == "serial":
            run_serial(args.serialport, args.baud, writer, fh)
        else:
            run_auto(args.host, args.port, args.serialport, args.baud, writer, fh)
    except KeyboardInterrupt:
        print("\n  Stopped by user.")
    finally:
        fh.close()
        log("CSV file closed. Done.")


if __name__ == "__main__":
    main()