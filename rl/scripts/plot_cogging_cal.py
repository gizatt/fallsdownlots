"""
Diagnostic plots for cogging feedforward LUT calibration data.

Usage:
    uv run scripts/plot_cogging_cal.py <csv>

Accepts either:
  - A dedicated cogging sweep CSV (from sysid.py --calibrate-cogging)
  - A regular sysid CSV (uses low-speed velocity_steps, same as build_cogging_lut.py)

Produces two figures:

  1. Raw voltage.q vs electrical angle
     Forward and reverse pass voltage.q plotted vs electrical angle in [0°, 360°).
     Offset between passes = friction + K_e·v (constant, should cancel cleanly).
     Shared angle-dependent shape = cogging.  Right panel shows the friction
     estimate; it should be roughly flat (not angle-dependent).

  2. Cogging profile and LUT
     Left: friction-cancelled average and the final zero-mean LUT.
     Right: LUT harmonic spectrum — dominant peak tells you which slot harmonic
     you're seeing (1× = fundamental electrical, 7× = fundamental mechanical cogging
     for a 7-pole-pair motor, etc.).
"""

import argparse
import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


N_POLE_PAIRS    = 7
N_LUT           = 128
MAX_SPEED_SYSID = 8.0  # rad/s


def load(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    tests = set(df["test"].unique())

    if "cogging_sweep" in tests:
        cal = df[df["test"] == "cogging_sweep"].copy()
        n_fwd = (cal["step_target"] > 0).sum()
        n_rev = (cal["step_target"] < 0).sum()
        print(f"[info] source: cogging sweep  {len(cal)} rows  "
              f"({n_fwd} fwd, {n_rev} rev)")
        return cal

    if "velocity_steps" in tests:
        sub = df[(df["test"] == "velocity_steps") &
                 (df["step_target"].abs() <= MAX_SPEED_SYSID)].copy()
        if sub.empty:
            sys.exit(f"ERROR: no velocity_steps rows with |speed| ≤ "
                     f"{MAX_SPEED_SYSID} rad/s")
        if "voltage_q" not in sub.columns:
            sys.exit("ERROR: voltage_q column missing — re-run sysid with "
                     "updated firmware that streams voltage.q")
        chunks = [grp.iloc[len(grp) // 2:]
                  for _, grp in sub.groupby("step_target")]
        cal = pd.concat(chunks)
        speeds = sorted(cal["step_target"].abs().unique())
        n_fwd = (cal["step_target"] > 0).sum()
        n_rev = (cal["step_target"] < 0).sum()
        print(f"[info] source: sysid velocity_steps  {len(cal)} rows  "
              f"({n_fwd} fwd, {n_rev} rev)  speeds={speeds}")
        return cal

    sys.exit(f"ERROR: no 'cogging_sweep' or 'velocity_steps' rows found in {path}")


def process(cal: pd.DataFrame) -> dict:
    fwd = cal[cal["step_target"] > 0].copy()
    rev = cal[cal["step_target"] < 0].copy()

    elec_fwd = (fwd["angle_rad"].values * N_POLE_PAIRS) % (2 * np.pi)
    elec_rev = (rev["angle_rad"].values * N_POLE_PAIRS) % (2 * np.pi)
    vq_fwd   = fwd["voltage_q"].values
    vq_rev   = rev["voltage_q"].values

    bin_edges   = np.linspace(0, 2 * np.pi, N_LUT + 1)
    bin_centers = 0.5 * (bin_edges[:-1] + bin_edges[1:])

    def bin_mean(elec, vq):
        means = np.full(N_LUT, np.nan)
        for i in range(N_LUT):
            mask = (elec >= bin_edges[i]) & (elec < bin_edges[i + 1])
            if mask.any():
                means[i] = vq[mask].mean()
        valid = ~np.isnan(means)
        if not valid.all():
            idx   = np.arange(N_LUT)
            means = np.interp(idx, idx[valid], means[valid], period=N_LUT)
        return means

    mean_fwd = bin_mean(elec_fwd, vq_fwd)
    mean_rev = bin_mean(elec_rev, vq_rev)
    avg      = 0.5 * (mean_fwd + mean_rev)
    lut      = avg - avg.mean()

    return dict(
        elec_fwd=elec_fwd, vq_fwd=vq_fwd,
        elec_rev=elec_rev, vq_rev=vq_rev,
        bin_centers=bin_centers,
        mean_fwd=mean_fwd, mean_rev=mean_rev,
        avg=avg, lut=lut,
    )


def figure_raw(d: dict, source_label: str):
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    fig.suptitle(f"voltage.q vs electrical angle  [{source_label}]", fontsize=13)

    deg = np.degrees(d["bin_centers"])

    ax = axes[0]
    ax.scatter(np.degrees(d["elec_fwd"]), d["vq_fwd"] * 1e3,
               s=1, alpha=0.15, color="C0", rasterized=True, label="forward")
    ax.scatter(np.degrees(d["elec_rev"]), d["vq_rev"] * 1e3,
               s=1, alpha=0.15, color="C1", rasterized=True, label="reverse")
    ax.plot(deg, d["mean_fwd"] * 1e3, lw=2, color="C0", label="fwd bin mean")
    ax.plot(deg, d["mean_rev"] * 1e3, lw=2, color="C1", label="rev bin mean")
    ax.axhline(0, color="gray", lw=0.5)
    ax.set_xlabel("electrical angle (deg)")
    ax.set_ylabel("voltage.q  (mV)")
    ax.set_title("Per-pass voltage vs electrical angle\n"
                 "offset = friction + K_e·v (cancels in avg); shared shape = cogging")
    ax.legend(fontsize=9, markerscale=8)
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    friction = 0.5 * (d["mean_fwd"] - d["mean_rev"])
    ax.plot(deg, friction * 1e3, lw=1.5, color="C4")
    ax.axhline(0, color="gray", lw=0.5)
    friction_rms = np.sqrt(np.mean(friction ** 2)) * 1e3
    friction_pk  = (friction.max() - friction.min()) * 1e3
    ax.set_xlabel("electrical angle (deg)")
    ax.set_ylabel("voltage  (mV)")
    ax.set_title(f"Friction + K_e·v estimate  = (fwd − rev) / 2\n"
                 f"RMS={friction_rms:.1f} mV  pk-pk={friction_pk:.1f} mV\n"
                 "should be roughly flat (not angle-dependent)")
    ax.grid(True, alpha=0.3)
    print(f"[info] friction+Ke·v  RMS={friction_rms:.1f} mV  pk-pk={friction_pk:.1f} mV")

    fig.tight_layout()


def figure_lut(d: dict, source_label: str):
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    fig.suptitle(f"Cogging profile and feedforward LUT  [{source_label}]", fontsize=13)

    deg = np.degrees(d["bin_centers"])

    ax = axes[0]
    ax.plot(deg, d["avg"] * 1e3, lw=1.5, color="C3",
            label="avg (fwd+rev)/2 — cogging + DC")
    ax.plot(deg, d["lut"] * 1e3, lw=2, color="C2",
            label="LUT (zero-mean feedforward)")
    ax.axhline(0, color="gray", lw=0.5)
    rms_mv = np.sqrt(np.mean(d["lut"] ** 2)) * 1e3
    pk_mv  = (d["lut"].max() - d["lut"].min()) * 1e3
    ax.set_xlabel("electrical angle (deg)")
    ax.set_ylabel("voltage  (mV)")
    ax.set_title(f"Cogging profile  RMS={rms_mv:.1f} mV  pk-pk={pk_mv:.1f} mV\n"
                 "smooth sinusoid → clean; jagged → need more data")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    print(f"[info] cogging LUT  RMS={rms_mv:.1f} mV  pk-pk={pk_mv:.1f} mV")

    ax = axes[1]
    spectrum = np.abs(np.fft.rfft(d["lut"]))
    freqs    = np.fft.rfftfreq(N_LUT, d=1.0 / N_LUT)
    ax.bar(freqs[1:], spectrum[1:], width=0.6, color="C2")
    ax.set_xlabel("harmonics (cycles per electrical revolution)")
    ax.set_ylabel("amplitude (linear)")
    ax.set_title("LUT spectrum\n"
                 "1× = fundamental electrical cogging\n"
                 f"{N_POLE_PAIRS}× = fundamental mechanical cogging")
    ax.set_xlim(0, 16)
    ax.grid(True, alpha=0.3)

    fig.tight_layout()


def main():
    parser = argparse.ArgumentParser(
        description="Plot cogging calibration diagnostics")
    parser.add_argument("csv", help="Cogging sweep or sysid CSV")
    args = parser.parse_args()

    cal = load(args.csv)

    source = ("cogging sweep" if "cogging_sweep" in cal["test"].values
              else "sysid velocity_steps")

    d = process(cal)
    figure_raw(d, source)
    figure_lut(d, source)
    plt.show()


if __name__ == "__main__":
    main()
