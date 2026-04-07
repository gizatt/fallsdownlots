"""
SysID data plotter and analysis.

Usage:
    uv run scripts/plot_sysid.py <sysid_data.csv>

Produces four figures:
    1. Step responses (torque mode) — velocity vs time, grouped by amplitude
    2. Step responses (velocity mode) — same
    3. Coast-down fits — exponential decay fit → damping time constant
    4. Bode plots (chirp) — magnitude and phase of vel/cmd transfer function,
       both torque and velocity mode overlaid

The Bode plot uses Welch's cross/auto-spectral density method, which handles
the log-chirp input well and gives a smooth estimate.
"""

import sys
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy import signal
from scipy.optimize import curve_fit


def load(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    df["t_s"] = df["t_us"] * 1e-6

    print("\n[sanity] sampling rate by test:")
    for test, group in df.groupby("test"):
        dt = np.diff(group["t_s"].values)
        fs = 1.0 / np.median(dt)
        gaps = dt[dt > 5.0 / fs]  # samples with >5x expected period
        print(f"  {test:<25s}  n={len(group):6d}  "
              f"fs={fs:6.1f} Hz  "
              f"min_dt={dt.min()*1e3:.2f}ms  max_dt={dt.max()*1e3:.2f}ms  "
              f"gaps(>5x)={len(gaps)}")
    print()

    return df


# ---------------------------------------------------------------------------
# Step response plots
# ---------------------------------------------------------------------------

def plot_steps(df: pd.DataFrame, test_label: str, cmd_unit: str, ax_title: str, axes):
    """Plot velocity responses for each step target on the provided axes."""
    sub = df[df["test"] == test_label].copy()
    if sub.empty:
        return

    targets = sorted(sub["step_target"].dropna().unique(), key=abs)
    colors = plt.cm.viridis(np.linspace(0.1, 0.9, len(targets)))

    for ax, (target, color) in zip(axes, zip(targets, colors)):
        chunk = sub[sub["step_target"] == target].copy()
        # Zero the time axis at the start of this step chunk.
        t = chunk["t_s"].values - chunk["t_s"].values[0]
        ax.plot(t, chunk["vel_rad_s"].values, color=color, lw=1.2)
        ax.axhline(target, color=color, lw=0.8, ls="--", alpha=0.6, label=f"cmd={target:+.1f} {cmd_unit}")
        ax.axhline(0, color="gray", lw=0.5)
        ax.set_ylabel("vel (rad/s)")
        ax.legend(loc="upper right", fontsize=8)
        ax.set_title(f"{ax_title}  cmd={target:+.1f} {cmd_unit}")

    axes[-1].set_xlabel("time (s)")


def figures_steps(df: pd.DataFrame):
    for label, unit, title in [
        ("torque_steps", "V", "Torque mode step"),
        ("velocity_steps", "rad/s", "Velocity mode step"),
    ]:
        sub = df[df["test"] == label]
        if sub.empty:
            continue
        targets = sorted(sub["step_target"].dropna().unique(), key=abs)
        n = len(targets)
        fig, axes = plt.subplots(n, 1, figsize=(10, 2.5 * n), sharex=False)
        if n == 1:
            axes = [axes]
        fig.suptitle(title + " responses", fontsize=13)
        plot_steps(df, label, unit, title, axes)
        fig.tight_layout()


# ---------------------------------------------------------------------------
# Coast-down: fit v(t) = v0 * exp(-t / tau)
# ---------------------------------------------------------------------------

def fit_coastdown(df: pd.DataFrame, test_label: str, title: str):
    sub = df[df["test"] == test_label].copy()
    if sub.empty:
        return

    # Find the transition from driven to coast: locate the last sample with
    # significant cmd, then find the first near-zero cmd after that.
    cmd = sub["cmd"].values
    driven = np.where(np.abs(cmd) > 0.1)[0]
    if len(driven) == 0:
        print(f"[analysis] no driven phase found in {test_label}")
        return
    after_driven = np.where(np.abs(cmd[driven[-1]:]) < 0.01)[0]
    if len(after_driven) == 0:
        print(f"[analysis] no coast phase found after driven phase in {test_label}")
        return
    coast_start_idx = driven[-1] + after_driven[0]

    coast = sub.iloc[coast_start_idx:].copy()
    t = coast["t_s"].values - coast["t_s"].values[0]
    v = coast["vel_rad_s"].values
    v0_guess = v[0]

    try:
        (v0, tau), _ = curve_fit(
            lambda t, v0, tau: v0 * np.exp(-t / tau),
            t, v,
            p0=[v0_guess, 0.5],
            bounds=([-np.inf, 0.01], [np.inf, 10.0]),
            maxfev=5000,
        )
    except RuntimeError:
        print(f"[analysis] coast-down fit failed for {test_label}")
        tau, v0 = float("nan"), v[0]

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(t, v, lw=1.2, label="measured vel")
    if np.isfinite(tau):
        ax.plot(t, v0 * np.exp(-t / tau), "--", lw=1.5,
                label=f"fit: v0={v0:.2f}, τ={tau:.3f}s\n(damping = J/τ)")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("vel (rad/s)")
    ax.set_title(f"{title} coast-down")
    ax.legend()
    fig.tight_layout()
    print(f"[analysis] {test_label}: v0={v0:.3f} rad/s, τ={tau:.4f} s")


# ---------------------------------------------------------------------------
# Bode plot from chirp data via Welch cross/auto-PSD
# ---------------------------------------------------------------------------

COHERENCE_THRESHOLD = 0.7  # Mask out frequencies below this coherence.
F_MIN = 0.1   # Hz — below this is DC drift noise.
F_MAX = 50.0  # Hz — hard cutoff; above this the chirp energy is too low to trust.


def compute_bode(df: pd.DataFrame, test_label: str):
    """Return (f, mag_db, phase_deg_unwrapped, coherence, good_mask) or None."""
    sub = df[df["test"] == test_label].copy().reset_index(drop=True)
    if sub.empty:
        return None

    dt = np.diff(sub["t_s"].values)
    fs = 1.0 / np.median(dt)

    u = sub["cmd"].values.astype(float)
    y = sub["vel_rad_s"].values.astype(float)

    nperseg = min(2048, len(u) // 4)
    f, Puu = signal.welch(u, fs=fs, nperseg=nperseg)
    _, Pyy = signal.welch(y, fs=fs, nperseg=nperseg)
    _, Puy = signal.csd(u, y, fs=fs, nperseg=nperseg)

    H = Puy / (Puu + 1e-12)
    mag_db = 20 * np.log10(np.abs(H) + 1e-12)
    coherence = np.abs(Puy) ** 2 / ((Puu * Pyy) + 1e-24)

    # Unwrap phase on the full array first, then mask — avoids wrap errors at
    # sparse coherent regions by working on the dense frequency grid.
    phase_deg = np.degrees(np.unwrap(np.angle(H)))

    good = (f >= F_MIN) & (f <= F_MAX) & (coherence > COHERENCE_THRESHOLD)

    in_range = (f >= F_MIN) & (f <= F_MAX)
    low_pct = 100 * (1 - coherence[in_range].gt(COHERENCE_THRESHOLD).mean()
                     if hasattr(coherence, 'gt')
                     else 1 - (coherence[in_range] > COHERENCE_THRESHOLD).mean())
    print(f"[analysis] {test_label}: fs={fs:.0f} Hz, "
          f"{low_pct:.0f}% of {F_MIN}-{F_MAX} Hz masked (coherence < {COHERENCE_THRESHOLD})")

    return f, mag_db, phase_deg, coherence, good


def bode_from_chirp(df: pd.DataFrame, test_label: str, label: str, ax_mag, ax_phase):
    result = compute_bode(df, test_label)
    if result is None:
        return
    f, mag_db, phase_deg, _, good = result
    mask = good & (f > 0.1)
    ax_mag.semilogx(f[mask], mag_db[mask], lw=1.5, label=label)
    ax_phase.semilogx(f[mask], phase_deg[mask], lw=1.5, label=label)


# ---------------------------------------------------------------------------
# Automated parameter extraction
# ---------------------------------------------------------------------------

def _first_order_plus_delay_phase(f, tau_m, tau_delay):
    """Phase in degrees of a first-order system with transport delay."""
    return -np.degrees(np.arctan(2 * np.pi * f * tau_m)) - 360 * f * tau_delay


def extract_parameters(df: pd.DataFrame):
    J_wheel = 1.2e-5  # kg·m², from geometry (0.5 * 0.020 * 0.035²)

    print("\n" + "=" * 60)
    print("EXTRACTED PARAMETERS")
    print("=" * 60)

    # --- Torque mode plant ---
    result = compute_bode(df, "torque_chirp")
    if result is not None:
        f, mag_db, phase_deg, _, good = result

        # DC gain: mean of magnitude at lowest coherent frequencies.
        dc_mask = good & (f < 2.0)
        if dc_mask.any():
            dc_gain_db = mag_db[dc_mask].mean()
            dc_gain = 10 ** (dc_gain_db / 20)
            print(f"\nTorque mode plant (V → rad/s):")
            print(f"  DC gain:        {dc_gain_db:.1f} dB  ({dc_gain:.2f} rad/s/V)")
        else:
            dc_gain_db = mag_db[good].max()
            dc_gain = 10 ** (dc_gain_db / 20)
            print(f"\nTorque mode plant (V → rad/s):")
            print(f"  DC gain (est):  {dc_gain_db:.1f} dB  ({dc_gain:.2f} rad/s/V)")

        # -3 dB corner frequency.
        threshold = dc_gain_db - 3.0
        crossings = np.where(good & (mag_db < threshold))[0]
        if len(crossings):
            f_3db = f[crossings[0]]
            tau_m = 1.0 / (2 * np.pi * f_3db)
            print(f"  -3 dB corner:   {f_3db:.2f} Hz  (τ_m = {tau_m*1000:.1f} ms)")
        else:
            f_3db, tau_m = None, None
            print(f"  -3 dB corner:   not found in coherent range")

        # Fit phase to first-order + delay model.
        fit_mask = good & (f > 0.5)
        if fit_mask.sum() > 5:
            p0 = [tau_m if tau_m else 0.05, 0.01]
            try:
                (tau_m_fit, tau_delay), _ = curve_fit(
                    _first_order_plus_delay_phase,
                    f[fit_mask], phase_deg[fit_mask],
                    p0=p0,
                    bounds=([1e-4, 0], [2.0, 0.1]),
                )
                print(f"  Phase fit τ_m:  {tau_m_fit*1000:.1f} ms  "
                      f"(corner = {1/(2*np.pi*tau_m_fit):.2f} Hz)")
                print(f"  Phase fit delay:{tau_delay*1000:.1f} ms")
            except RuntimeError:
                print(f"  Phase fit:      failed")

    # --- Coastdown: damping ---
    tau_coast = None
    sub_cd = df[df["test"] == "torque_coastdown"].copy()
    if not sub_cd.empty:
        cmd = sub_cd["cmd"].values
        driven = np.where(np.abs(cmd) > 0.1)[0]
        if len(driven):
            after = np.where(np.abs(cmd[driven[-1]:]) < 0.01)[0]
            if len(after):
                coast = sub_cd.iloc[driven[-1] + after[0]:]
                t = coast["t_s"].values - coast["t_s"].values[0]
                v = coast["vel_rad_s"].values
                try:
                    (v0, tau_coast), _ = curve_fit(
                        lambda t, v0, tau: v0 * np.exp(-t / tau),
                        t, v, p0=[v[0], 0.05],
                        bounds=([-np.inf, 0.001], [np.inf, 5.0]),
                    )
                    damping = J_wheel / tau_coast
                    print(f"\nCoast-down (torque mode):")
                    print(f"  τ_coast:        {tau_coast*1000:.1f} ms")
                    print(f"  Damping (J/τ):  {damping:.2e} N·m·s/rad  "
                          f"(J_wheel={J_wheel:.2e})")
                    print(f"  → MuJoCo joint damping: {damping:.2e}")
                except RuntimeError:
                    print(f"\nCoast-down fit: failed")

    # --- Velocity mode bandwidth and resonance ---
    result_v = compute_bode(df, "velocity_chirp")
    if result_v is not None:
        f, mag_db, phase_deg, _, good = result_v
        print(f"\nVelocity mode closed loop (rad/s → rad/s):")

        dc_mask = good & (f < 2.0)
        if dc_mask.any():
            dc_db = mag_db[dc_mask].mean()
        else:
            dc_db = 0.0

        # Bandwidth.
        bw_mask = good & (mag_db < dc_db - 3.0)
        if bw_mask.any():
            print(f"  Bandwidth (-3dB):{f[bw_mask][0]:.2f} Hz")
        else:
            print(f"  Bandwidth:       > coherent range")

        # Resonance peak (search only within valid range, above 0.5 Hz).
        search = good & (f > 0.5)
        if search.any():
            peak_idx = mag_db[search].argmax()
            peak_f = f[search][peak_idx]
            peak_db = mag_db[search][peak_idx]
            if peak_db > dc_db + 0.5:
                print(f"  Resonance peak:  {peak_f:.2f} Hz  ({peak_db - dc_db:+.1f} dB above DC)")
            else:
                print(f"  No significant resonance peak")

    # --- Step: linearity check ---
    for label, unit in [("torque_steps", "V"), ("velocity_steps", "rad/s")]:
        sub = df[df["test"] == label]
        if sub.empty:
            continue
        print(f"\nStep gains ({label}):")
        targets = sorted(sub["step_target"].dropna().unique(), key=abs)
        for tgt in targets:
            if tgt <= 0:
                continue
            chunk = sub[sub["step_target"] == tgt]
            # Steady-state: last 40% of samples.
            ss = chunk.iloc[int(len(chunk) * 0.6):]
            gain = ss["vel_rad_s"].mean() / tgt
            print(f"  cmd={tgt:5.1f} {unit}  →  ss_vel={ss['vel_rad_s'].mean():6.2f} rad/s  "
                  f"gain={gain:.3f} (rad/s)/{unit}")

    print("=" * 60 + "\n")


def figure_cogging(df: pd.DataFrame, n_pole_pairs: int = 7, max_speed: float = 8.0):
    """
    Diagnose low-speed velocity oscillation by plotting velocity error vs rotor angle.

    Cogging torque repeats every 1/n_pole_pairs of a mechanical revolution, so
    folding the angle by 2π/n_pole_pairs and plotting velocity error against it
    will reveal a clear sinusoidal pattern if cogging is the cause.

    Uses steady-state portions of low-speed velocity steps (|cmd| <= max_speed).
    """
    sub = df[(df["test"] == "velocity_steps") &
             (df["step_target"].abs() <= max_speed)].copy()
    if sub.empty:
        print("[cogging] no low-speed velocity_steps data found")
        return

    # Use only the steady-state tail of each step (last 50%).
    chunks = []
    for tgt, grp in sub.groupby("step_target"):
        tail = grp.iloc[len(grp) // 2:]
        chunks.append(tail)
    sub = pd.concat(chunks).sort_values("t_s").reset_index(drop=True)

    vel_error = sub["vel_rad_s"].values - sub["step_target"].values
    angle_raw = sub["angle_rad"].values
    angle_unwrapped = np.unwrap(angle_raw)

    mech_angle   = angle_unwrapped % (2 * np.pi)
    elec_angle   = angle_unwrapped % (2 * np.pi / n_pole_pairs)

    n_bins = 60
    bin_edges = np.linspace(0, 2 * np.pi / n_pole_pairs, n_bins + 1)
    bin_centers = 0.5 * (bin_edges[:-1] + bin_edges[1:])
    bin_idx = np.clip(np.digitize(elec_angle, bin_edges) - 1, 0, n_bins - 1)

    fwd_mask = sub["step_target"].values > 0
    rev_mask = ~fwd_mask

    def bin_mean_for(mask):
        return np.array([
            vel_error[mask & (bin_idx == i)].mean()
            if (mask & (bin_idx == i)).any() else np.nan
            for i in range(n_bins)
        ])

    bin_mean_fwd = bin_mean_for(fwd_mask)
    bin_mean_rev = bin_mean_for(rev_mask)

    fig, axes = plt.subplots(1, 3, figsize=(15, 4))
    fig.suptitle(f"Low-speed velocity error (<= {max_speed} rad/s) — cogging diagnosis", fontsize=12)

    # Time domain.
    t = sub["t_s"].values - sub["t_s"].values[0]
    axes[0].plot(t[fwd_mask], vel_error[fwd_mask], lw=0.8, alpha=0.7, label="fwd")
    axes[0].plot(t[rev_mask], vel_error[rev_mask], lw=0.8, alpha=0.7, label="rev")
    axes[0].axhline(0, color="gray", lw=0.5)
    axes[0].set_xlabel("time (s)")
    axes[0].set_ylabel("velocity error (rad/s)")
    axes[0].set_title("Time domain")
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.3)

    # Full mechanical revolution.
    axes[1].scatter(mech_angle[fwd_mask], vel_error[fwd_mask], s=2, alpha=0.3,
                    rasterized=True, label="fwd")
    axes[1].scatter(mech_angle[rev_mask], vel_error[rev_mask], s=2, alpha=0.3,
                    rasterized=True, label="rev")
    axes[1].axhline(0, color="gray", lw=0.5)
    axes[1].set_xlabel("mechanical angle (rad, mod 2π)")
    axes[1].set_ylabel("velocity error (rad/s)")
    axes[1].set_title("vs mechanical angle")
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.3)

    # Electrical angle (folded by pole pairs) — cogging appears here.
    # If fwd and rev bin-means align: pure cogging (position-dependent).
    # If they're mirrored/shifted: friction asymmetry or encoder issue.
    axes[2].scatter(elec_angle[fwd_mask], vel_error[fwd_mask], s=2, alpha=0.15,
                    color="C0", rasterized=True)
    axes[2].scatter(elec_angle[rev_mask], vel_error[rev_mask], s=2, alpha=0.15,
                    color="C1", rasterized=True)
    axes[2].plot(bin_centers, bin_mean_fwd, color="C0", lw=2, label="fwd mean")
    axes[2].plot(bin_centers, bin_mean_rev, color="C1", lw=2, label="rev mean")
    axes[2].axhline(0, color="gray", lw=0.5)
    axes[2].set_xlabel(f"electrical angle (rad, mod 2π/{n_pole_pairs})")
    axes[2].set_ylabel("velocity error (rad/s)")
    axes[2].set_title(f"vs electrical angle (folded × {n_pole_pairs})\n"
                      "fwd≈rev → cogging; mirrored → friction/encoder")
    axes[2].legend(fontsize=8)
    axes[2].grid(True, alpha=0.3)

    pp = n_pole_pairs
    osc_freq_hz = sub["step_target"].abs().median() * pp / (2 * np.pi)
    print(f"[cogging] at {sub['step_target'].abs().median():.1f} rad/s, "
          f"expected cogging frequency: {osc_freq_hz:.2f} Hz  "
          f"(= speed × {pp} pole_pairs / 2π)")
    peak_error = np.abs(vel_error).mean()
    print(f"[cogging] mean |vel_error|: {peak_error:.3f} rad/s")

    fig.tight_layout()



def figure_velocity_error_spectra(df: pd.DataFrame, n_pole_pairs: int = 7):
    """
    Plot velocity error power spectrum for each commanded speed.

    Cogging signature: dominant peak frequency scales linearly with speed
      (f_cogging = speed * n_pole_pairs / 2π).
    Integrator limit cycle: dominant peak stays at roughly the same frequency
      regardless of speed.
    Encoder quantization: broadband noise floor that rises at low speeds.
    """
    sub = df[df["test"] == "velocity_steps"].copy()
    if sub.empty:
        return

    targets = sorted(sub["step_target"].dropna().unique(), key=abs)
    pos_targets = [t for t in targets if t > 0]

    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    fig.suptitle("Velocity error spectra — oscillation source diagnosis", fontsize=12)

    colors = plt.cm.viridis(np.linspace(0.1, 0.9, len(pos_targets)))
    peak_freqs = []

    for tgt, color in zip(pos_targets, colors):
        # Use steady-state tail of positive step only.
        chunk = sub[sub["step_target"] == tgt].copy()
        tail = chunk.iloc[len(chunk) // 2:]
        if len(tail) < 64:
            continue

        dt = np.median(np.diff(tail["t_s"].values))
        fs = 1.0 / dt
        vel_err = (tail["vel_rad_s"] - tail["step_target"]).values

        f, psd = signal.welch(vel_err, fs=fs, nperseg=min(256, len(vel_err) // 2))
        f_cogging = tgt * n_pole_pairs / (2 * np.pi)

        axes[0].semilogy(f, psd, color=color, lw=1.5, label=f"{tgt:.0f} rad/s")
        axes[0].axvline(f_cogging, color=color, lw=0.8, ls="--", alpha=0.6)

        # Record the dominant peak frequency (excluding DC).
        dc_cut = f > 0.3
        if dc_cut.any():
            peak_f = f[dc_cut][psd[dc_cut].argmax()]
            peak_freqs.append((tgt, peak_f, f_cogging))

    axes[0].set_xlabel("frequency (Hz)")
    axes[0].set_ylabel("PSD [(rad/s)²/Hz]")
    axes[0].set_title("Velocity error PSD per speed\n(dashed = expected cogging freq)")
    axes[0].legend(fontsize=8)
    axes[0].grid(True, which="both", alpha=0.3)

    # Second panel: measured peak freq vs speed — should be linear for cogging.
    if peak_freqs:
        speeds, peaks, cog_freqs = zip(*peak_freqs)
        axes[1].plot(speeds, peaks, "o-", lw=1.5, label="measured peak")
        axes[1].plot(speeds, cog_freqs, "--", color="gray", lw=1.5,
                     label=f"cogging ({n_pole_pairs} pole pairs)")
        axes[1].set_xlabel("commanded speed (rad/s)")
        axes[1].set_ylabel("dominant oscillation frequency (Hz)")
        axes[1].set_title("Peak frequency vs speed\n"
                          "linear through origin → cogging\n"
                          "flat → integrator/PID limit cycle")
        axes[1].legend(fontsize=9)
        axes[1].grid(True, alpha=0.3)

        for spd, pf, cf in peak_freqs:
            print(f"[spectra] {spd:5.1f} rad/s: peak={pf:.2f} Hz, "
                  f"cogging_expected={cf:.2f} Hz, "
                  f"ratio={pf/cf:.2f}")

    fig.tight_layout()


def figure_bode(df: pd.DataFrame):
    fig, (ax_mag, ax_phase) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    fig.suptitle("Bode plot: cmd → velocity", fontsize=13)

    bode_from_chirp(df, "torque_chirp",   "torque mode (V → rad/s)",   ax_mag, ax_phase)
    bode_from_chirp(df, "velocity_chirp", "velocity mode (rad/s → rad/s)", ax_mag, ax_phase)

    ax_mag.set_ylabel("magnitude (dB)")
    ax_mag.axhline(-3, color="gray", lw=0.8, ls="--", alpha=0.5, label="-3 dB")
    ax_mag.legend(fontsize=9)
    ax_mag.grid(True, which="both", alpha=0.3)

    ax_phase.set_ylabel("phase (deg)")
    ax_phase.set_xlabel("frequency (Hz)")
    ax_phase.axhline(-90, color="gray", lw=0.8, ls="--", alpha=0.5)
    ax_phase.axhline(-180, color="gray", lw=0.8, ls="--", alpha=0.5)
    ax_phase.legend(fontsize=9)
    ax_phase.grid(True, which="both", alpha=0.3)

    fig.tight_layout()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", help="Path to sysid CSV file")
    args = parser.parse_args()

    df = load(args.csv)
    print(f"[analysis] loaded {len(df)} rows, tests: {df['test'].unique().tolist()}")

    extract_parameters(df)

    figures_steps(df)
    figure_bode(df)
    figure_cogging(df)
    figure_velocity_error_spectra(df)

    for label, title in [
        ("torque_coastdown",   "Torque mode"),
        ("velocity_coastdown", "Velocity mode"),
    ]:
        fit_coastdown(df, label, title)

    plt.show()


if __name__ == "__main__":
    main()
