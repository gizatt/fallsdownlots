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

    def run_ecc_calibration(
        self,
        voltage: float = 3.0,
        duration: float = 8.0,
    ) -> list[dict]:
        """
        Phase 1 — eccentricity calibration.

        Spin at a high-ish open-loop torque voltage so the motor moves fast
        enough that cogging averages out.  Detrending the angle ramp reveals
        the AS5600 eccentricity error as a sinusoid at 1× mechanical.

        After running, feed the printed ECC_A / ECC_PHI into the firmware
        calibration block in main-sysid.cpp / main-fallsdownlots.cpp and
        reflash before running phase 2.
        """
        self.set_torque_mode()
        time.sleep(0.2)
        print(f"[host] -> phase 1: eccentricity cal  voltage={voltage:.2f} V  dur={duration:.1f}s")
        self._reader.flush()
        self.set_target(voltage)
        time.sleep(duration)
        self.set_target(0.0)
        rows = self._reader.collect()
        for r in rows:
            r["test"] = "eccentricity_cal"
            r["step_target"] = voltage
        print(f"[host]   collected {len(rows)} rows")
        return rows

    def run_cogging_calibration(
        self,
        voltage: float = 0.3,
        duration: float = 15.0,
    ) -> list[dict]:
        """
        Phase 2 — cogging + ezero calibration.

        Requires eccentricity correction already flashed into firmware.
        Spin at a low open-loop torque voltage so the motor moves slowly
        and cogging dominates the velocity ripple.  plot_sysid.py bins
        velocity vs corrected electrical angle to reveal the cogging profile.

        initFOC runs at firmware boot with the corrected sensor, so the
        zero_electric_angle printed at startup is the ezero to use.
        """
        self.set_torque_mode()
        time.sleep(0.2)
        print(f"[host] -> phase 2: cogging cal  voltage={voltage:.2f} V  dur={duration:.1f}s")
        self._reader.flush()
        self.set_target(voltage)
        time.sleep(duration)
        self.set_target(0.0)
        rows = self._reader.collect()
        for r in rows:
            r["test"] = "cogging_cal"
            r["step_target"] = voltage
        print(f"[host]   collected {len(rows)} rows")
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
    cal_group = parser.add_mutually_exclusive_group()
    cal_group.add_argument("--calibrate-ecc", action="store_true",
                           help="Phase 1: eccentricity calibration (high-speed torque spin). "
                                "Fit ECC_A/ECC_PHI, paste into firmware, reflash, then run --calibrate-cogging.")
    cal_group.add_argument("--calibrate-cogging", action="store_true",
                           help="Phase 2: cogging calibration (low-speed torque spin). "
                                "Requires eccentricity correction already in firmware.")
    parser.add_argument("--ecc-voltage", type=float, default=3.0,
                        help="Open-loop voltage for eccentricity phase (default 3.0 V).")
    parser.add_argument("--cogging-voltage", type=float, default=0.3,
                        help="Open-loop voltage for cogging phase (default 0.3 V).")
    args = parser.parse_args()

    port = normalize_port(args.port)
    print(f"[host] connecting to {port} ...")
    board = SysIDBoard(port, args.baud)
    print("[host] connected.")

    if args.calibrate_ecc or args.calibrate_cogging:
        out_path = Path(args.out) if args.out else \
            Path(f"cal_{time.strftime('%Y%m%d_%H%M%S')}.csv")
        out_path.parent.mkdir(parents=True, exist_ok=True)
        try:
            if args.calibrate_ecc:
                rows = board.run_ecc_calibration(voltage=args.ecc_voltage)
                print(f"\n[host] Analyse with: uv run scripts/plot_sysid.py {out_path}")
                print(f"[host] Then paste ECC_A / ECC_PHI into firmware and reflash.")
                print(f"[host] Then run: python scripts/sysid.py --port <port> --calibrate-cogging")
            else:
                rows = board.run_cogging_calibration(voltage=args.cogging_voltage)
                print(f"\n[host] Analyse with: uv run scripts/plot_sysid.py {out_path}")
        finally:
            board.close()
        save_rows(rows, out_path)
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
