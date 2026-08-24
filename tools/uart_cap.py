"""Capture the ASCLIN0 debug console (115200 8N1) to stdout and a log file.

Bring-up usage: start this in the background BEFORE flashing, so the reset
banner and the boot dumps land in the capture instead of scrolling past.

    python tools/uart_cap.py                     # auto-detect, log to tools/uart.log
    python tools/uart_cap.py --port COM4
    python tools/uart_cap.py --out bringup.log --quiet

The board's debug bridge enumerates as "Infineon DAS JDS COM"; --port is only
needed when several are attached. Runs until Ctrl-C.
"""
import argparse
import sys
import time

import serial
from serial.tools import list_ports

DEFAULT_BAUD = 115200


def autodetect():
    """Return the COM port of the Infineon DAS bridge, or None."""
    candidates = []
    for p in list_ports.comports():
        blob = f"{p.description} {p.manufacturer or ''} {p.product or ''}".lower()
        if "das" in blob or "jds" in blob or "infineon" in blob:
            candidates.append(p.device)
    if candidates:
        return candidates[0]
    ports = [p.device for p in list_ports.comports()]
    return ports[0] if len(ports) == 1 else None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="COM port (default: auto-detect the DAS bridge)")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--out", default="tools/uart.log", help="log file (appended)")
    ap.add_argument("--quiet", action="store_true", help="log file only, no stdout")
    args = ap.parse_args()

    port = args.port or autodetect()
    if port is None:
        avail = ", ".join(p.device for p in list_ports.comports()) or "none"
        sys.exit(f"No serial port found (available: {avail}). Pass --port.")

    try:
        ser = serial.Serial(port, args.baud, timeout=0.2)
    except serial.SerialException as e:
        sys.exit(f"Could not open {port}: {e}\n"
                 f"Another capture, the GUI or a terminal is probably holding it.")

    print(f"--- capturing {port} @ {args.baud} -> {args.out} (Ctrl-C to stop) ---",
          file=sys.stderr)

    with open(args.out, "a", encoding="utf-8", errors="replace") as log:
        log.write(f"\n--- capture started {time.strftime('%Y-%m-%d %H:%M:%S')} "
                  f"on {port} @ {args.baud} ---\n")
        log.flush()
        buf = bytearray()
        try:
            while True:
                chunk = ser.read(4096)
                if not chunk:
                    continue
                buf.extend(chunk)
                # emit whole lines only, so a timestamp marks the line's arrival
                while b"\n" in buf:
                    raw, _, rest = buf.partition(b"\n")
                    buf = bytearray(rest)
                    line = raw.decode("utf-8", errors="replace").rstrip("\r")
                    stamped = f"[{time.strftime('%H:%M:%S')}] {line}"
                    log.write(stamped + "\n")
                    log.flush()          # flush every line: a hang must not eat the tail
                    if not args.quiet:
                        print(stamped, flush=True)
        except KeyboardInterrupt:
            print("\n--- capture stopped ---", file=sys.stderr)
        finally:
            ser.close()


if __name__ == "__main__":
    main()
