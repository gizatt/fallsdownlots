"""
SysID driver for the fallsdownlots motor.

Connects to the sysid firmware over serial, runs test sequences and saves
results to CSV.

Usage:
    # Encoder LUT calibration (run once per motor before sysid):
    python scripts/sysid.py --port /dev/ttyACM0 --calibrate [--cal-voltage 3.0]

    # Full sysid sequence (steps + chirp in velocity mode):
    python scripts/sysid.py --port /dev/ttyACM0 [--out data/sysid_<timestamp>.csv]

Calibration workflow:
    1. python scripts/sysid.py --port <port> --calibrate --out cal_right.csv
    2. python scripts/build_encoder_lut.py cal_right.csv r > src/encoder_lut_r.h
    3. Repeat for the left motor, write to encoder_lut_l.h
    4. Reflash firmware (pio run -e fallsdownlots -t upload)
    5. Run sysid normally

Encoder calibration output is a CSV with columns:
    test, direction, ref_elec, raw_mech

Normal sysid output is a CSV with columns:
    test, t_us, recv_time, angle_rad, vel_rad_s, cmd, step_target
"""

import argparse
import csv
import queue
import re
import sys
import threading
import time
from pathlib import Path

import serial


def normalize_port(port: str) -> str:
    """Translate COM<N> -> /dev/ttyS<N> when running under WSL."""
    m = re.fullmatch(r"COM(\d+)", port, re.IGNORECASE)
    if m and sys.platform != "win32":
        return f"/dev/ttyS{m.group(1)}"
    return port


# ---------------------------------------------------------------------------
# Serial reader — background thread, buffers incoming rows.
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

                # Encoder calibration data: E,<F|R>,<ref_elec>,<raw_mech>
                if line.startswith("E,"):
                    parts = line.split(",")
                    if len(parts) == 4:
                        row = {
                            "test":      "encoder_cal",
                            "direction": parts[1],
                            "ref_elec":  float(parts[2]),
                            "raw_mech":  float(parts[3]),
                        }
                        with self._lock:
                            self._rows.append(row)
                    continue

                # Normal sysid data: <t_us>,<angle_rad>,<vel_rad_s>,<cmd>
                parts = line.split(",")
                if len(parts) == 4:
                    row = {
                        "t_us":       int(parts[0]),
                        "angle_rad":  float(parts[1]),
                        "vel_rad_s":  float(parts[2]),
                        "cmd":        float(parts[3]),
                        "recv_time":  time.monotonic(),
                    }
                    with self._lock:
                        self._rows.append(row)

            except (ValueError, UnicodeDecodeError):
                pass
            except serial.SerialException:
                break

    def flush(self):
        with self._lock:
            self._rows.clear()

    def collect(self) -> list[dict]:
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
        self._ser = serial.Serial(
            port, baud, timeout=1.0,
            dsrdtr=False, rtscts=False,
        )
        time.sleep(2.0)
        self._ser.reset_input_buffer()
        self._reader = SerialReader(self._ser)

    def wait_for_data(self, timeout: float = 10.0) -> bool:
        print(f"[host] waiting for firmware data stream (timeout={timeout}s)...")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._reader._lock:
                if self._reader._rows:
                    print(f"[host] firmware is live.")
                    return True
            time.sleep(0.2)
        print("[host] ERROR: no data received. Is the sysid firmware flashed?")
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
        self._reader.flush()
        self.start_chirp(amp, f0, f1, dur)
        time.sleep(dur + 1.0)
        self.set_target(0.0)
        rows = self._reader.collect()
        for r in rows:
            r["test"] = test_label
            r["step_target"] = float("nan")
        return rows

    def run_encoder_calibration(self, voltage: float = 3.0) -> list[dict]:
        """
        Ben Katz encoder LUT calibration.

        Steps the motor's D-axis (stepper-style) through one full mechanical
        rotation forward then backward, recording raw encoder vs reference
        angle at each step.  Takes ~10 s at default settings.

        Feed the resulting CSV to build_encoder_lut.py to produce the C header.
        """
        # How long the firmware cal takes: 2 * CAL_STEPS * CAL_SETTLE_US
        # = 2 * 1000 * 5ms = 10 s, plus some margin.
        expected_duration_s = 12.0
        print(f"[host] -> encoder cal  voltage={voltage:.2f} V  "
              f"(~{expected_duration_s:.0f}s)")
        self._reader.flush()
        self._send(f"K{voltage:.2f}")

        deadline = time.monotonic() + expected_duration_s + 10.0
        while time.monotonic() < deadline:
            try:
                msg = self._reader._comment_queue.get(timeout=1.0)
                if "CAL_DONE" in msg:
                    break
                if "CAL_REVERSE" in msg:
                    print("[host]   reversing direction...")
            except queue.Empty:
                pass

        rows = self._reader.collect()
        print(f"[host]   collected {len(rows)} calibration rows")
        return rows

    def close(self):
        self._reader.stop()
        self._ser.close()


# ---------------------------------------------------------------------------
# CSV output
# ---------------------------------------------------------------------------

SYSID_FIELDS = ["test", "t_us", "recv_time", "angle_rad", "vel_rad_s", "cmd", "step_target"]
CAL_FIELDS   = ["test", "direction", "ref_elec", "raw_mech"]

def save_rows(rows: list[dict], path: Path, fields: list[str], append: bool = False):
    mode = "a" if append else "w"
    with open(path, mode, newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        if not append:
            writer.writeheader()
        writer.writerows(rows)
    print(f"[host] saved {len(rows)} rows -> {path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="SysID / encoder-cal driver")
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyACM0 or COM5")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--out", default=None, help="Output CSV path")
    parser.add_argument("--calibrate", action="store_true",
                        help="Run encoder LUT calibration instead of sysid")
    parser.add_argument("--cal-voltage", type=float, default=3.0,
                        help="D-axis voltage for calibration (default 3.0 V)")
    args = parser.parse_args()

    port = normalize_port(args.port)
    print(f"[host] connecting to {port} ...")
    board = SysIDBoard(port, args.baud)
    print("[host] connected.")

    try:
        if args.calibrate:
            out_path = Path(args.out) if args.out else \
                Path(f"cal_{time.strftime('%Y%m%d_%H%M%S')}.csv")
            out_path.parent.mkdir(parents=True, exist_ok=True)
            rows = board.run_encoder_calibration(voltage=args.cal_voltage)
            save_rows(rows, out_path, CAL_FIELDS)
            print(f"\n[host] Next steps:")
            print(f"[host]   python scripts/build_encoder_lut.py {out_path} r > src/encoder_lut_r.h")
            print(f"[host]   (or 'l' for left motor)")
            print(f"[host]   Then reflash: pio run -e fallsdownlots -t upload")
            return

        # Normal sysid.
        out_path = Path(args.out) if args.out else \
            Path(f"sysid_{time.strftime('%Y%m%d_%H%M%S')}.csv")
        out_path.parent.mkdir(parents=True, exist_ok=True)

        if not board.wait_for_data():
            sys.exit(1)

        all_rows: list[dict] = []

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

        save_rows(all_rows, out_path, SYSID_FIELDS)
        print(f"[host] done. {len(all_rows)} total rows.")

    except KeyboardInterrupt:
        print("\n[host] interrupted.")
        if args.calibrate:
            return
        board.set_target(0.0)
    finally:
        board.close()


if __name__ == "__main__":
    main()
