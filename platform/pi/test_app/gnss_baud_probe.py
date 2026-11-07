#!/usr/bin/env python3
"""gnss_baud_probe.py - find out which UART carries the GNSS and at which baud rate.

1. Checks the Raspberry Pi UART configuration (config.txt, cmdline.txt, which
   UART is on GPIO14/15, Bluetooth, serial console).
2. Reads the raw serial port at each candidate baud rate and reports which
   rate gives NMEA sentences with valid checksums (or which binary protocol).

The serdev GNSS driver must NOT own the UART while probing, otherwise the
tty device does not exist. Temporarily comment out dtoverlay=neo6m-serdev
in config.txt and reboot, then:

    sudo python3 gnss_baud_probe.py                 # config check + probe /dev/serial0
    sudo python3 gnss_baud_probe.py --port /dev/ttyAMA0
    sudo python3 gnss_baud_probe.py --check-only    # config check, works with the driver loaded

Standard library only.
"""
from __future__ import annotations

import argparse
import os
import re
import select
import sys
import termios
import time
from pathlib import Path

RATES = [9600, 115200, 38400, 57600, 19200, 230400, 460800, 921600, 4800]
NMEA_RE = re.compile(rb"\$([A-Z]{2}[A-Z]{3}),[^\r\n*$]*\*([0-9A-Fa-f]{2})")


def check_config() -> None:
    print("=== Raspberry Pi UART configuration ===")
    cfg = next((p for p in (Path("/boot/firmware/config.txt"), Path("/boot/config.txt")) if p.exists()), None)
    if cfg:
        lines = [l.strip() for l in cfg.read_text(errors="replace").splitlines()]
        active = [l for l in lines if l and not l.startswith("#")]
        rel = [l for l in active if re.search(r"uart|bt|blue|serdev|neo6m|gpio=", l, re.I)]
        print(f"{cfg}:")
        for l in rel or ["  (no UART related lines)"]:
            print("   ", l)
        keys = " ".join(active)
        if re.search(r"\benable-uart\b", keys):
            print("  !! 'enable-uart' is not a valid key: use enable_uart=1")
        if not re.search(r"\benable_uart\s*=\s*1", keys):
            print("  !! enable_uart=1 missing")
        if re.search(r"disable-ble\b", keys):
            print("  !! 'dtoverlay=disable-ble' does not exist: use dtoverlay=disable-bt")
        if not re.search(r"dtoverlay\s*=\s*(disable-bt|pi3-disable-bt|miniuart-bt|pi3-miniuart-bt)", keys):
            print("  !! no disable-bt / miniuart-bt overlay: on a Pi 3 the PL011 (uart0, ttyAMA0) is wired to")
            print("     Bluetooth, NOT to GPIO14/15. The neo6m overlay targets uart0, so the driver hears Bluetooth.")
            print("     Fix: add 'dtoverlay=disable-bt' and run 'sudo systemctl disable hciuart', then reboot.")
    else:
        print("config.txt not found")
    cmd = Path("/proc/cmdline").read_text(errors="replace") if Path("/proc/cmdline").exists() else ""
    if re.search(r"console=(serial0|ttyAMA0|ttyS0)", cmd):
        print("  !! serial console active on the UART (/proc/cmdline). Remove 'console=serial0,...' from cmdline.txt")
        print("     (or sudo raspi-config -> Interface -> Serial: login shell NO, hardware YES).")
    for link in ("/dev/serial0", "/dev/serial1"):
        if os.path.islink(link):
            print(f"  {link} -> {os.path.realpath(link)}  ({'GPIO14/15' if link.endswith('0') else 'Bluetooth side'})")
    print("  hint: serial0 must be ttyAMA0 for the neo6m overlay (it targets uart0)\n")


def set_raw(fd: int, rate: int) -> bool:
    const = getattr(termios, f"B{rate}", None)
    if const is None:
        return False
    a = termios.tcgetattr(fd)
    a[0] = 0                                            # iflag
    a[1] = 0                                            # oflag
    a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL # cflag: 8N1
    a[3] = 0                                            # lflag
    a[4] = a[5] = const
    a[6][termios.VMIN] = 0
    a[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, a)
    termios.tcflush(fd, termios.TCIFLUSH)
    return True


def nmea_ok(m: re.Match) -> bool:
    body = m.group(0)[1:m.group(0).index(b"*")]
    cs = 0
    for c in body:
        cs ^= c
    return cs == int(m.group(2), 16)


def probe(port: str, secs: float) -> None:
    print(f"=== probing {port}, {secs:.1f} s per rate ===")
    fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    best = None
    try:
        for rate in RATES:
            if not set_raw(fd, rate):
                print(f"{rate:>7}: not supported by termios")
                continue
            buf = bytearray()
            t_end = time.monotonic() + secs
            while time.monotonic() < t_end:
                r, _, _ = select.select([fd], [], [], 0.1)
                if r:
                    buf += os.read(fd, 4096)
            good = [m for m in NMEA_RE.finditer(buf) if nmea_ok(m)]
            types = sorted({m.group(1).decode() for m in good})
            printable = sum(32 <= b < 127 or b in (10, 13) for b in buf) / max(1, len(buf))
            extra = ""
            if b"\xb5\x62" in buf:
                extra += " UBX-binary"
            if b"\xf1\xd9" in buf:
                extra += " Allystar-binary"
            print(f"{rate:>7}: {len(buf):6d} bytes, {100 * printable:5.1f}% printable, "
                  f"{len(good):3d} valid NMEA {types[:6]}{extra}")
            if good and (best is None or len(good) > best[1]):
                best = (rate, len(good), good[0].group(0).decode(errors="replace"))
    finally:
        os.close(fd)
    print()
    if best:
        print(f"==> GNSS NMEA at {best[0]} baud, e.g. {best[2][:80]}")
        print(f"    driver: sudo insmod neo6m_gnss_serdev.ko baud={best[0]}   (or leave auto-detect)")
    else:
        print("==> no NMEA at any rate. If bytes arrived but nothing is printable at any rate, this port")
        print("    is not the GNSS (Bluetooth?) or TX/RX are not connected GNSS-TX -> Pi-RX (GPIO15, pin 10).")
        print("    If 0 bytes at every rate: no signal on RX at all (wiring, power, GNSS UART disabled).")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/serial0")
    ap.add_argument("--secs", type=float, default=2.5)
    ap.add_argument("--check-only", action="store_true")
    args = ap.parse_args()
    check_config()
    if args.check_only:
        return 0
    if not os.path.exists(args.port):
        print(f"{args.port} does not exist. If the neo6m serdev overlay is active it hides the tty:")
        print("comment out 'dtoverlay=neo6m-serdev' in config.txt, reboot, run this again.")
        return 1
    probe(args.port, args.secs)
    return 0


if __name__ == "__main__":
    sys.exit(main())
