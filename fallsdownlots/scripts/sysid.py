"""
SysID driver for the fallsdownlots motor.

Connects to the sysid firmware over serial, runs step and chirp test sequences
in both torque and velocity control modes, and saves the results to CSV.

Usage:
    python scripts/sysid.py --port /dev/ttyACM0 [--out data/sysid_<timestamp>.csv]

Each test sequence is appended to a single CSV file with a 'test' column
identifying the run. Comment lines from the firmware (starting with #) are
printed to stdout but not saved.

Test sequences run (in order):
    Torque mode:
        - Steps: +/-[0.5, 1.0, 2.0, 3.5] V, 2s on / 1s settle
        - Coast-down: 3.5V for 2s then 0V for 3s
        - Log-chirp: 3.0V amp, 0.2-80 Hz, 40s

    Velocity mode (SimpleFOC velocity PID):
        - Same structure but in rad/s: +/-[5, 10, 20, 40] rad/s
        - Chirp: 30 rad/s amp, 0.2-30 Hz, 40s

Firmware must be flashed with the 'sysid' PlatformIO env.
"""

import argparse
import csv
import sys
import threading
import time
from pathlib import Path

import serial

# ---------------------------------------------------------------------------
# Serial reader — runs in a background thread, buffers incoming data rows.
# ---------------------------------------------------------------------------

class SerialReader:
    def __init__(self, ser: serial.Serial):
        self._ser = ser
        self._rows: list[dict] = []
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            try:
                raw = self._ser.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", errors="replace").strip()
                if line.startswith("#"):
                    print(f"[fw] {line[1:].strip()}")
                    continue
                parts = line.split(",")
                if len(parts) != 4:
                    continue
                row = {
                    "t_us": int(parts[0]),
                    "angle_rad": float(parts[1]),
                    "vel_rad_s": float(parts[2]),
                    "cmd": float(parts[3]),
                    "recv_time": time.monotonic(),
                }
                with self._lock:
                    self._rows.append(row)
            except (ValueError, UnicodeDecodeError):
                pass
            except serial.SerialException:
                break

    def flush(self):
        """Discard all buffered rows (call before starting a test)."""
        with self._lock:
            self._rows.clear()

    def collect(self) -> list[dict]:
        """Return and clear all buffered rows since the last flush/collect."""
        with self._lock:
            rows, self._rows = self._rows, []
        return rows

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2)


# ---------------------------------------------------------------------------
# Board interface
# ---------------------------------------------------------------------------

class SysIDBoard:
    def __init__(self, port: str, baud: int = 115200):
        self._ser = serial.Serial(port, baud, timeout=1.0)
        time.sleep(2.0)  # Wait for board reset after USB connect.
        self._ser.reset_input_buffer()
        self._reader = SerialReader(self._ser)

    def _send(self, cmd: str):
        self._ser.write((cmd + "\n").encode("ascii"))

    def set_torque_mode(self):
        print("[host] -> torque mode")
        self._send("M0")
        time.sleep(0.1)

    def set_velocity_mode(self, P=0.25, I=5.0, D=0.001, Tf=0.02):
        print(f"[host] -> velocity mode  P={P} I={I} D={D} Tf={Tf}")
        self._send("M1")
        self._send(f"P{P}")
        self._send(f"I{I}")
        self._send(f"D{D}")
        self._send(f"F{Tf}")
        time.sleep(0.1)

    def set_target(self, val: float):
        self._send(f"T{val:.4f}")

    def start_chirp(self, amp: float, f0: float, f1: float, dur: float):
        print(f"[host] -> chirp amp={amp} f0={f0} f1={f1} dur={dur}s")
        self._send(f"C{amp},{f0},{f1},{dur}")

    def run_step_test(
        self,
        amplitudes: list[float],
        on_duration: float,
        settle_duration: float,
        test_label: str,
    ) -> list[dict]:
        """Step through each amplitude (positive then negative), collect data."""
        rows = []
        for amp in amplitudes:
            for sign in (+1, -1):
                val = sign * amp
                print(f"[host] step {val:+.2f}  ({on_duration}s)")
                self._reader.flush()
                self.set_target(val)
                time.sleep(on_duration)
                self.set_target(0.0)
                chunk = self._reader.collect()
                for r in chunk:
                    r["test"] = test_label
                    r["step_target"] = val
                rows.extend(chunk)
                time.sleep(settle_duration)
        return rows

    def run_coastdown_test(
        self,
        drive_val: float,
        drive_duration: float,
        coast_duration: float,
        test_label: str,
    ) -> list[dict]:
        """Drive to steady speed, then cut to 0 and record deceleration."""
        print(f"[host] coast-down: drive={drive_val:.2f} for {drive_duration}s "
              f"then coast {coast_duration}s")
        self._reader.flush()
        self.set_target(drive_val)
        time.sleep(drive_duration)
        self.set_target(0.0)
        time.sleep(coast_duration)
        rows = self._reader.collect()
        for r in rows:
            r["test"] = test_label
            r["step_target"] = 0.0
        return rows

    def run_chirp_test(
        self,
        amp: float,
        f0: float,
        f1: float,
        dur: float,
        test_label: str,
    ) -> list[dict]:
        """Run a firmware-generated log-chirp and collect the response."""
        self._reader.flush()
        self.start_chirp(amp, f0, f1, dur)
        time.sleep(dur + 1.0)  # Extra second to collect the tail.
        self.set_target(0.0)
        rows = self._reader.collect()
        for r in rows:
            r["test"] = test_label
            r["step_target"] = float("nan")
        return rows

    def close(self):
        self._reader.stop()
        self._ser.close()


# ---------------------------------------------------------------------------
# CSV output
# ---------------------------------------------------------------------------

FIELDS = ["test", "t_us", "recv_time", "angle_rad", "vel_rad_s", "cmd", "step_target"]

def save_rows(rows: list[dict], path: Path, append: bool = False):
    mode = "a" if append else "w"
    with open(path, mode, newline="") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS, extrasaction="ignore")
        if not append:
            writer.writeheader()
        writer.writerows(rows)
    print(f"[host] saved {len(rows)} rows -> {path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="SysID driver")
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--out", default=None,
                        help="Output CSV path. Defaults to sysid_<timestamp>.csv")
    args = parser.parse_args()

    out_path = Path(args.out) if args.out else \
        Path(f"sysid_{time.strftime('%Y%m%d_%H%M%S')}.csv")
    out_path.parent.mkdir(parents=True, exist_ok=True)

    print(f"[host] connecting to {args.port} ...")
    board = SysIDBoard(args.port, args.baud)
    print("[host] connected.")

    all_rows: list[dict] = []

    try:
        # ------------------------------------------------------------------
        # Torque (voltage) mode
        # ------------------------------------------------------------------
        board.set_torque_mode()
        time.sleep(0.5)

        all_rows += board.run_step_test(
            amplitudes=[0.5, 1.0, 2.0, 3.5],
            on_duration=2.0,
            settle_duration=1.0,
            test_label="torque_steps",
        )

        all_rows += board.run_coastdown_test(
            drive_val=3.5,
            drive_duration=2.0,
            coast_duration=3.0,
            test_label="torque_coastdown",
        )

        all_rows += board.run_chirp_test(
            amp=3.0, f0=0.2, f1=80.0, dur=40.0,
            test_label="torque_chirp",
        )

        # ------------------------------------------------------------------
        # Velocity mode
        # ------------------------------------------------------------------
        board.set_velocity_mode(P=0.25, I=5.0, D=0.001, Tf=0.02)
        time.sleep(0.5)

        all_rows += board.run_step_test(
            amplitudes=[5.0, 10.0, 20.0, 40.0],
            on_duration=2.0,
            settle_duration=1.0,
            test_label="velocity_steps",
        )

        all_rows += board.run_coastdown_test(
            drive_val=30.0,
            drive_duration=2.0,
            coast_duration=3.0,
            test_label="velocity_coastdown",
        )

        all_rows += board.run_chirp_test(
            amp=30.0, f0=0.2, f1=30.0, dur=40.0,
            test_label="velocity_chirp",
        )

    except KeyboardInterrupt:
        print("\n[host] interrupted, saving what we have...")
    finally:
        board.set_target(0.0)
        board.close()

    save_rows(all_rows, out_path)
    print(f"[host] done. {len(all_rows)} total rows.")


if __name__ == "__main__":
    main()
