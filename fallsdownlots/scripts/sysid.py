"""
SysID driver for the fallsdownlots motor.

Connects to the sysid firmware over serial, runs step and chirp test sequences
in both torque and velocity control modes, and saves the results to CSV.

Usage:
    python scripts/sysid.py --port /dev/ttyACM0 [--out data/sysid_<timestamp>.csv]
    python scripts/sysid.py --port COM5 [--out data/sysid_<timestamp>.csv]

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
import re
import sys
import threading
import time
from pathlib import Path

import queue
import serial


def normalize_port(port: str) -> str:
    """Translate COM<N> -> /dev/ttyS<N> when running under WSL/Linux."""
    m = re.fullmatch(r"COM(\d+)", port, re.IGNORECASE)
    if m and sys.platform != "win32":
        return f"/dev/ttyS{m.group(1)}"
    return port

# ---------------------------------------------------------------------------
# Serial reader — runs in a background thread, buffers incoming data rows.
# ---------------------------------------------------------------------------

class SerialReader:
    def __init__(self, ser: serial.Serial):
        self._ser = ser
        self._rows: list[dict] = []
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._comment_queue: queue.Queue[str] = queue.Queue()
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
                    msg = line[1:].strip()
                    print(f"[fw] {msg}")
                    self._comment_queue.put(msg)
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
        # dsrdtr=False, rtscts=False: prevent flow-control lines from
        # toggling DTR and resetting the nRF52840 on connect (Windows issue).
        self._ser = serial.Serial(
            port, baud, timeout=1.0,
            dsrdtr=False, rtscts=False,
        )
        time.sleep(2.0)  # Wait for board to be ready.
        self._ser.reset_input_buffer()
        self._reader = SerialReader(self._ser)

    def wait_for_data(self, timeout: float = 10.0) -> bool:
        """Block until data rows are flowing in, or timeout. Returns True if data arrived."""
        print(f"[host] waiting for firmware data stream (timeout={timeout}s)...")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._reader._lock:
                if self._reader._rows:
                    rate_est = len(self._reader._rows)
                    print(f"[host] receiving data ({rate_est} rows in first check). Firmware is live.")
                    return True
            time.sleep(0.2)
        print("[host] ERROR: no data received from firmware. Is the sysid firmware flashed?")
        return False

    def _send(self, cmd: str):
        self._ser.write((cmd + "\r\n").encode("ascii"))

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

    def calibrate_zero(self, voltage: float = 2.0) -> dict | None:
        """
        Run the Z command: open-loop phase voltage at 4 cardinal electrical angles,
        parse the firmware's implied_zero measurements, and return a dict with:
            zero_electric_angle  float  (radians) — pass to motor.initFOC()
            direction            str    "CW" or "CCW"
            measurements         list of dicts per cardinal angle
        Returns None on failure (no measurements received).
        """
        print(f"[host] -> zero calibration  voltage={voltage:.2f} V  (~{4 * 0.7:.1f}s)")

        measurements = []
        zero_angle = None
        direction = None

        # Drain any stale comment lines that arrived before this call.
        while not self._reader._comment_queue.empty():
            self._reader._comment_queue.get_nowait()

        self._send(f"Z{voltage:.2f}")

        # SerialReader owns the port — read comment lines via its queue.
        # Firmware takes 600ms per angle + 100ms gap × 4 + a little slack.
        total_wait = 4 * 0.8 + 2.0
        deadline = time.monotonic() + total_wait
        while time.monotonic() < deadline:
            try:
                msg = self._reader._comment_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            # Parse: zero_meas: cmd_elec=X mech=Y elec_meas=Z implied_zero=W
            if "zero_meas:" in msg:
                parts = {}
                for token in msg.split("zero_meas:")[1].split():
                    k, _, v = token.partition("=")
                    try:
                        parts[k] = float(v)
                    except ValueError:
                        pass
                if "implied_zero" in parts:
                    measurements.append(parts)
            # Parse: zero_cal result: zero_electric_angle=X  direction=Y
            elif "zero_cal result:" in msg:
                for token in msg.split("zero_cal result:")[1].split():
                    k, _, v = token.partition("=")
                    if k == "zero_electric_angle":
                        try:
                            zero_angle = float(v)
                        except ValueError:
                            pass
                    elif k == "direction":
                        direction = v.strip()
                break  # Done.

        if zero_angle is None or not measurements:
            print("[host] ERROR: zero calibration failed — no result received.")
            return None

        print(f"\n[host] === Zero calibration result ===")
        print(f"[host]   zero_electric_angle = {zero_angle:.5f} rad")
        print(f"[host]   direction           = {direction}")
        print(f"[host]   ({len(measurements)} measurements, std = "
              f"{self._circular_std([m['implied_zero'] for m in measurements]):.4f} rad)")
        print(f"[host] Paste into initFOC:")
        print(f"[host]   motor.initFOC({zero_angle:.5f}f, Direction::{direction});")
        print()

        return {"zero_electric_angle": zero_angle, "direction": direction,
                "measurements": measurements}

    @staticmethod
    def _circular_std(angles: list[float]) -> float:
        import math
        s = sum(math.sin(a) for a in angles)
        c = sum(math.cos(a) for a in angles)
        r = math.sqrt(s*s + c*c) / len(angles)
        return math.sqrt(-2 * math.log(max(r, 1e-9)))

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
    parser.add_argument("--calibrate-zero", action="store_true",
                        help="Run electrical zero calibration only (Z command), then exit.")
    parser.add_argument("--cal-voltage", type=float, default=2.0,
                        help="Open-loop voltage to use during zero calibration (default 2.0 V).")
    args = parser.parse_args()

    port = normalize_port(args.port)
    print(f"[host] connecting to {port} ...")
    board = SysIDBoard(port, args.baud)
    print("[host] connected.")

    if args.calibrate_zero:
        try:
            board.calibrate_zero(voltage=args.cal_voltage)
        finally:
            board.close()
        return

    out_path = Path(args.out) if args.out else \
        Path(f"sysid_{time.strftime('%Y%m%d_%H%M%S')}.csv")
    out_path.parent.mkdir(parents=True, exist_ok=True)

    if not board.wait_for_data():
        board.close()
        sys.exit(1)

    all_rows: list[dict] = []

    try:
        # ------------------------------------------------------------------
        # Torque (voltage) mode
        # ------------------------------------------------------------------
        # board.set_torque_mode()
        # time.sleep(0.5)

        # all_rows += board.run_step_test(
        #     amplitudes=[0.5, 1.0, 2.0, 3.5],
        #     on_duration=2.0,
        #     settle_duration=1.0,
        #     test_label="torque_steps",
        # )

        # all_rows += board.run_coastdown_test(
        #     drive_val=3.5,
        #     drive_duration=2.0,
        #     coast_duration=3.0,
        #     test_label="torque_coastdown",
        # )

        # all_rows += board.run_chirp_test(
        #     amp=3.0, f0=0.2, f1=80.0, dur=40.0,
        #     test_label="torque_chirp",
        # )

        # ------------------------------------------------------------------
        # Velocity mode
        # ------------------------------------------------------------------
        board.set_velocity_mode(P=0.03, I=1, D=0.0, Tf=0.01)
        time.sleep(0.5)

        all_rows += board.run_step_test(
            amplitudes=[2.5, 5.0, 8.0, 10.0, 20.0, 40.0, 80.0],
            on_duration=5.0,
            settle_duration=1.0,
            test_label="velocity_steps",
        )

        all_rows += board.run_coastdown_test(
            drive_val=80.0,
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
