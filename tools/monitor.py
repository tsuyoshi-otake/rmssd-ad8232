"""Real-time logger for the rmssd-ad8232 firmware.

Reads the unified I/S/R/E serial stream from the M5Stack Basic, splits it into
per-stream CSV files under data/session_<timestamp>/, and points data/latest at
the active session so Claude Code / Codex can always Read the same path.

Default behaviour:
  * Open the configured serial port at 460800 bps.
  * Send "TIME <unix_sec>" so the firmware tags Unix timestamps on its CSV rows.
  * Decode each line:
        I,<session_ms>,<unix_ms>,<event>[,<param>]   -> events.csv
        S,<session_ms>,<unix_ms>,<bpm>,<rmssd>,<base>,<calibn>,<frozen>,<drops>,<leads_off>
                                                     -> summary.csv  (every 500 ms)
        R,<session_ms>,<unix_ms>,<rr_ms>,<bpm>,<rmssd>,<base>,<ratio>,<rr_count>,<leads_off>
                                                     -> rr.csv        (per R-wave)
        E,<session_ms>,<raw>                         -> ecg.csv       (250 Hz)
  * Echo every parsed line back to stdout so you can also tail the terminal.

Stop with Ctrl+C; CSVs are flushed continuously (line-buffered) so partial
sessions are still readable.

Usage:
    python tools/monitor.py                     # COM3 + auto time sync
    python tools/monitor.py --port COM4
    python tools/monitor.py --no-time-sync      # skip the TIME command
    python tools/monitor.py --no-ecg            # tell the firmware to stop E rows
    python tools/monitor.py --data-dir D:/hrv   # custom output root
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import serial  # pyserial


SUMMARY_HEADER = [
    "wall_iso", "session_ms", "unix_ms",
    "bpm", "rmssd", "base", "calibn", "frozen", "drops", "leads_off",
]
RR_HEADER = [
    "wall_iso", "session_ms", "unix_ms",
    "rr_ms", "bpm", "rmssd", "base", "ratio", "rr_count", "leads_off",
]
ECG_HEADER = ["wall_iso", "session_ms", "raw"]
EVENT_HEADER = ["wall_iso", "session_ms", "unix_ms", "event", "param"]
BREATH_HEADER = [
    "wall_iso", "session_ms", "unix_ms",
    "brpm", "brpm_conf", "brpm_source",   # 0=NONE,1=FM,2=AM,3=FUSED
    "rmssd_norm",
    "brpm_fm", "rqi_fm", "brpm_am", "rqi_am",
]


def update_latest_pointer(data_dir: Path, session_name: str) -> None:
    """Make data/latest point at the new session.

    Symlinks on Windows need Developer Mode or admin; fall back to a text file
    so AI tooling can still find the active session.
    """
    latest = data_dir / "latest"
    try:
        if latest.is_symlink() or latest.exists():
            if latest.is_symlink() or latest.is_file():
                latest.unlink()
            else:
                # latest is a real dir from a previous run; leave it alone.
                pass
        latest.symlink_to(session_name, target_is_directory=True)
        return
    except (OSError, NotImplementedError):
        pass
    (data_dir / "LATEST.txt").write_text(session_name + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=os.environ.get("RMSSD_PORT", "COM3"))
    p.add_argument("--baud", type=int, default=460800)
    p.add_argument("--data-dir", default="data")
    p.add_argument("--no-time-sync", action="store_true",
                   help="do not send TIME <unix_sec> on startup")
    p.add_argument("--no-ecg", action="store_true",
                   help='send "ECG OFF" so the firmware skips 250 Hz E rows')
    p.add_argument("--quiet", action="store_true",
                   help="do not echo parsed lines to stdout")
    return p.parse_args()


def main() -> int:
    args = parse_args()

    session_tag = datetime.now().strftime("%Y%m%d_%H%M%S")
    data_dir = Path(args.data_dir)
    session_dir = data_dir / f"session_{session_tag}"
    session_dir.mkdir(parents=True, exist_ok=True)
    update_latest_pointer(data_dir, session_dir.name)

    # line-buffered text files: rows survive Ctrl+C.
    summary_f = (session_dir / "summary.csv").open("w", newline="", encoding="utf-8", buffering=1)
    rr_f      = (session_dir / "rr.csv").open(     "w", newline="", encoding="utf-8", buffering=1)
    ecg_f     = (session_dir / "ecg.csv").open(    "w", newline="", encoding="utf-8", buffering=1)
    events_f  = (session_dir / "events.csv").open( "w", newline="", encoding="utf-8", buffering=1)
    breath_f  = (session_dir / "breath.csv").open( "w", newline="", encoding="utf-8", buffering=1)

    summary_w = csv.writer(summary_f); summary_w.writerow(SUMMARY_HEADER)
    rr_w      = csv.writer(rr_f);      rr_w.writerow(RR_HEADER)
    ecg_w     = csv.writer(ecg_f);     ecg_w.writerow(ECG_HEADER)
    events_w  = csv.writer(events_f);  events_w.writerow(EVENT_HEADER)
    breath_w  = csv.writer(breath_f);  breath_w.writerow(BREATH_HEADER)

    print(f"[host] session dir: {session_dir}", file=sys.stderr)
    print(f"[host] opening {args.port} @ {args.baud}", file=sys.stderr)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        print(f"[host] could not open {args.port}: {e}", file=sys.stderr)
        return 2

    with ser:
        # Optionally tell the firmware our wall-clock time and ECG preference.
        if not args.no_time_sync:
            unix = int(time.time())
            ser.write(f"TIME {unix}\n".encode())
            print(f"[host] -> TIME {unix}", file=sys.stderr)
        if args.no_ecg:
            ser.write(b"ECG OFF\n")
            print("[host] -> ECG OFF", file=sys.stderr)

        try:
            while True:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                if not line:
                    continue
                wall = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
                parts = line.split(",")
                kind = parts[0]

                try:
                    if kind == "E" and len(parts) >= 3:
                        ecg_w.writerow([wall, parts[1], parts[2]])
                    elif kind == "S" and len(parts) >= 10:
                        summary_w.writerow([wall] + parts[1:10])
                    elif kind == "R" and len(parts) >= 10:
                        rr_w.writerow([wall] + parts[1:10])
                    elif kind == "B" and len(parts) >= 11:
                        breath_w.writerow([wall] + parts[1:11])
                    elif kind == "I" and len(parts) >= 4:
                        ev = parts[3]
                        param = ",".join(parts[4:]) if len(parts) > 4 else ""
                        events_w.writerow([wall, parts[1], parts[2], ev, param])
                    else:
                        events_w.writerow([wall, "", "", "RAW", line])
                except Exception as e:  # pragma: no cover - keep logging despite parse errors
                    events_w.writerow([wall, "", "", "PARSE_ERR", f"{e}: {line}"])

                if not args.quiet:
                    print(line)
        except KeyboardInterrupt:
            print("\n[host] stopping (Ctrl+C)", file=sys.stderr)

    summary_f.close(); rr_f.close(); ecg_f.close(); events_f.close(); breath_f.close()
    print(f"[host] CSVs flushed in {session_dir}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
