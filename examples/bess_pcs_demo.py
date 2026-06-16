"""
examples/bess_pcs_demo.py
=========================
Simulate a 1 MW / 2 MWh BESS PCS with P-f and Q-V droop control over 60 s.

Scenario
--------
t =  0–20 s : Grid stable at 60 Hz, 1.0 pu.  BESS dispatched at 400 kW export.
t = 20–40 s : Frequency drops linearly to 59.5 Hz (large load event on island).
              Droop controller ramps BESS output toward 1 MW.
              A reactive load also causes voltage to sag to 0.96 pu.
t = 40–60 s : Frequency recovers back to 60 Hz.  Voltage also recovers.

The simulation uses a simple forward-Euler integration loop — there is no
switching-level model.  The PCS operates as a grid-forming unit with 5 % P-f
and 5 % Q-V droop.

Outputs
-------
- Console table with key variables every 5 s
- A four-panel matplotlib figure saved to examples/bess_pcs_demo.png
"""

import math
import sys
import os

# Allow running from repo root or examples/ directory
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
import matplotlib
matplotlib.use("Agg")          # headless rendering for CI / scripts
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

from pcs.converter import BidirectionalConverter
from pcs.control.droop import DroopController
from pcs.protection import PCSProtection


# ---------------------------------------------------------------------------
# System parameters
# ---------------------------------------------------------------------------

RATED_KW       = 1_000.0     # 1 MW
RATED_KVAR     = 600.0       # 0.6 MVAr
RATED_FREQ     = 60.0        # Hz
DC_VOLTAGE     = 800.0       # V
AC_VOLTAGE     = 690.0       # V  (MV transformer secondary)
SW_FREQ        = 10_000.0    # Hz

SOC_INITIAL    = 0.60        # 60 % state of charge
CAPACITY_KWH   = 2_000.0     # 2 MWh

P_DISPATCH_KW  = 400.0       # initial active power set-point
Q_DISPATCH_KVAR = 0.0        # initial reactive power set-point

DROOP_P_PCT    = 5.0         # P-f droop %
DROOP_Q_PCT    = 5.0         # Q-V droop %

T_END          = 60.0        # simulation duration (s)
DT             = 0.1         # time step (s) — adequate for droop dynamics

# ---------------------------------------------------------------------------
# Build profile arrays
# ---------------------------------------------------------------------------

t  = np.arange(0.0, T_END + DT, DT)
N  = len(t)

# Frequency profile: nominal → dip → recovery
f_grid = np.where(
    t < 20,
    60.0,
    np.where(
        t < 40,
        60.0 - 0.5 * (t - 20) / 20.0,   # linear dip to 59.5 Hz at t=40
        59.5 + 0.5 * (t - 40) / 20.0,   # linear recovery to 60 Hz at t=60
    )
)

# Voltage profile: nominal → sag → recovery
v_pcc_pu = np.where(
    t < 20,
    1.00,
    np.where(
        t < 40,
        1.00 - 0.04 * (t - 20) / 20.0,   # sag to 0.96 pu
        0.96 + 0.04 * (t - 40) / 20.0,   # recovery to 1.0 pu
    )
)

# ---------------------------------------------------------------------------
# Instantiate PCS components
# ---------------------------------------------------------------------------

converter  = BidirectionalConverter(RATED_KW, DC_VOLTAGE, AC_VOLTAGE, SW_FREQ)
droop_ctrl = DroopController(
    rated_kw=RATED_KW,
    rated_kvar=RATED_KVAR,
    rated_freq_hz=RATED_FREQ,
    rated_voltage_pu=1.0,
)
protection = PCSProtection()

# ---------------------------------------------------------------------------
# Pre-allocate result arrays
# ---------------------------------------------------------------------------

p_out_kw   = np.zeros(N)
q_out_kvar = np.zeros(N)
eta        = np.zeros(N)
i_dc_A     = np.zeros(N)
i_ac_A     = np.zeros(N)
soc        = np.zeros(N)
trip_flag  = np.zeros(N, dtype=bool)

soc_k = SOC_INITIAL

# ---------------------------------------------------------------------------
# Simulation loop
# ---------------------------------------------------------------------------

for k in range(N):
    fk = float(f_grid[k])
    vk = float(v_pcc_pu[k])

    # Droop control
    p_k = droop_ctrl.frequency_droop(fk, P_DISPATCH_KW, DROOP_P_PCT)
    q_k = droop_ctrl.voltage_droop(vk, Q_DISPATCH_KVAR, DROOP_Q_PCT)

    p_out_kw[k]   = p_k
    q_out_kvar[k] = q_k

    # Converter metrics
    lf = p_k / RATED_KW if RATED_KW > 0 else 0.0
    eta[k]   = converter.efficiency(lf) if lf > 0 else float("nan")
    i_dc_A[k] = converter.dc_current(p_k) if p_k > 0 else 0.0

    # Apparent current on AC side
    s_kva = math.sqrt(p_k**2 + q_k**2)
    pf = p_k / s_kva if s_kva > 0 else 1.0
    i_ac_A[k] = converter.ac_current_A(p_k, max(pf, 0.01)) if p_k > 0 else 0.0

    # SOC integration (simple Coulomb counting)
    # Energy exported this step: E = P * dt / 3600 kWh
    soc_k -= (p_k * DT) / (CAPACITY_KWH * 3_600.0)
    soc_k  = max(0.0, min(1.0, soc_k))
    soc[k] = soc_k

    # Protection evaluation
    prot = protection.evaluate_all(
        v_pu=vk, f_hz=fk, v_dc=DC_VOLTAGE, v_dc_max=880.0,
        i_meas=i_ac_A[k], i_max=1_200.0,
    )
    trip_flag[k] = prot["any_trip"]

# ---------------------------------------------------------------------------
# Console summary table
# ---------------------------------------------------------------------------

print()
print("=" * 80)
print(f"  BESS PCS Droop Simulation — {RATED_KW/1000:.0f} MW / {CAPACITY_KWH/1000:.0f} MWh")
print("=" * 80)
print(f"{'t (s)':>8} {'f (Hz)':>8} {'V (pu)':>8} {'P (kW)':>9} "
      f"{'Q (kvar)':>10} {'η (%)':>7} {'SOC (%)':>8} {'Trip':>6}")
print("-" * 80)

report_times = [0, 10, 20, 25, 30, 35, 40, 50, 60]
for rt in report_times:
    k = min(int(rt / DT), N - 1)
    eta_pct = eta[k] * 100 if not math.isnan(eta[k]) else 0.0
    print(
        f"{t[k]:8.1f} {f_grid[k]:8.3f} {v_pcc_pu[k]:8.3f} "
        f"{p_out_kw[k]:9.1f} {q_out_kvar[k]:10.1f} "
        f"{eta_pct:7.2f} {soc[k]*100:8.2f} {'TRIP' if trip_flag[k] else 'OK':>6}"
    )

print("=" * 80)
print(f"  Final SOC: {soc[-1]*100:.2f} %")
print(f"  Modulation index: {converter.modulation_index():.4f}")
print(f"  Peak AC current: {i_ac_A.max():.1f} A")
print(f"  Peak efficiency: {np.nanmax(eta)*100:.2f} %")
print()

# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------

fig = plt.figure(figsize=(14, 10))
fig.suptitle(
    f"BESS PCS Droop Control Simulation — {RATED_KW/1000:.0f} MW / {CAPACITY_KWH/1000:.0f} MWh",
    fontsize=13, fontweight="bold"
)
gs = gridspec.GridSpec(2, 2, figure=fig, hspace=0.42, wspace=0.32)

# Panel 1: Frequency & active power
ax1 = fig.add_subplot(gs[0, 0])
color_f = "#1f77b4"
color_p = "#d62728"
ax1b = ax1.twinx()
ax1.plot(t, f_grid,    color=color_f, lw=1.8, label="Grid frequency")
ax1b.plot(t, p_out_kw, color=color_p, lw=1.8, ls="--", label="P output")
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Frequency (Hz)", color=color_f)
ax1b.set_ylabel("Active power (kW)", color=color_p)
ax1.set_title("P–f Droop Response")
ax1.tick_params(axis="y", colors=color_f)
ax1b.tick_params(axis="y", colors=color_p)
ax1.axhline(RATED_FREQ, color=color_f, lw=0.7, ls=":")
lines1 = [
    plt.Line2D([0], [0], color=color_f, lw=1.8),
    plt.Line2D([0], [0], color=color_p, lw=1.8, ls="--"),
]
ax1.legend(lines1, ["f (Hz)", "P (kW)"], loc="lower left", fontsize=8)

# Panel 2: Voltage & reactive power
ax2 = fig.add_subplot(gs[0, 1])
color_v = "#2ca02c"
color_q = "#ff7f0e"
ax2b = ax2.twinx()
ax2.plot(t, v_pcc_pu,    color=color_v, lw=1.8, label="PCC voltage")
ax2b.plot(t, q_out_kvar, color=color_q, lw=1.8, ls="--", label="Q output")
ax2.set_xlabel("Time (s)")
ax2.set_ylabel("Voltage (pu)", color=color_v)
ax2b.set_ylabel("Reactive power (kvar)", color=color_q)
ax2.set_title("Q–V Droop Response")
ax2.tick_params(axis="y", colors=color_v)
ax2b.tick_params(axis="y", colors=color_q)
ax2.axhline(1.0, color=color_v, lw=0.7, ls=":")
lines2 = [
    plt.Line2D([0], [0], color=color_v, lw=1.8),
    plt.Line2D([0], [0], color=color_q, lw=1.8, ls="--"),
]
ax2.legend(lines2, ["V (pu)", "Q (kvar)"], loc="lower left", fontsize=8)

# Panel 3: SOC & efficiency
ax3 = fig.add_subplot(gs[1, 0])
color_soc = "#9467bd"
color_eta = "#8c564b"
ax3b = ax3.twinx()
ax3.plot(t, soc * 100, color=color_soc, lw=1.8, label="SOC")
ax3b.plot(t, np.where(np.isnan(eta), 0, eta * 100), color=color_eta, lw=1.8, ls="--", label="η")
ax3.set_xlabel("Time (s)")
ax3.set_ylabel("State of charge (%)", color=color_soc)
ax3b.set_ylabel("Efficiency (%)", color=color_eta)
ax3.set_title("SOC & Converter Efficiency")
ax3.tick_params(axis="y", colors=color_soc)
ax3b.tick_params(axis="y", colors=color_eta)
lines3 = [
    plt.Line2D([0], [0], color=color_soc, lw=1.8),
    plt.Line2D([0], [0], color=color_eta, lw=1.8, ls="--"),
]
ax3.legend(lines3, ["SOC (%)", "η (%)"], loc="upper right", fontsize=8)

# Panel 4: AC & DC currents
ax4 = fig.add_subplot(gs[1, 1])
ax4.plot(t, i_ac_A, color="#e377c2", lw=1.8, label="AC current (A)")
ax4.plot(t, i_dc_A, color="#7f7f7f", lw=1.8, ls="--", label="DC current (A)")
ax4.set_xlabel("Time (s)")
ax4.set_ylabel("Current (A)")
ax4.set_title("Converter AC / DC Currents")
ax4.legend(fontsize=8)
ax4.axhline(1_200, color="#e377c2", lw=0.7, ls=":", label="AC limit")

# Trip markers on all panels
for ax in [ax1, ax2, ax3, ax4]:
    for k in np.where(trip_flag)[0]:
        ax.axvspan(t[k] - DT / 2, t[k] + DT / 2, color="red", alpha=0.15)

out_path = os.path.join(os.path.dirname(__file__), "bess_pcs_demo.png")
fig.savefig(out_path, dpi=150, bbox_inches="tight")
print(f"Plot saved to: {out_path}")
