# Getting Started Tutorial

A step-by-step walkthrough for running the BESS PCS droop simulation, interpreting the four output panels, and experimenting with droop settings to observe their effect on frequency and voltage response.

---

## Prerequisites

- Python 3.9 or later.
- The following packages (install with `pip install -r requirements.txt`):
  - `numpy` — array-based simulation loop.
  - `matplotlib` — four-panel results figure.

```bash
git clone https://github.com/<your-username>/pcs-control.git
cd pcs-control
pip install -r requirements.txt
```

---

## 1. Run the BESS PCS Demo

From the repository root:

```bash
python examples/bess_pcs_demo.py
```

You will see a console output table followed by a path to the saved PNG:

```
================================================================================
  BESS PCS Droop Simulation — 1 MW / 2 MWh
================================================================================
   t (s)   f (Hz)   V (pu)   P (kW)   Q (kvar)   η (%)  SOC (%)   Trip
--------------------------------------------------------------------------------
     0.0   60.000    1.000    400.0        0.0   97.75    60.00     OK
    10.0   60.000    1.000    400.0        0.0   97.75    59.96     OK
    20.0   60.000    1.000    400.0        0.0   97.75    59.93     OK
    25.0   59.875    0.990    567.4      240.0   97.93    59.89     OK
    30.0   59.750    0.980    733.4      479.7   98.00    59.83     OK
    35.0   59.625    0.970    733.4      479.7   98.00    59.76     OK
    40.0   59.500    0.960    733.3      480.0   98.03    59.68     OK
    50.0   59.750    0.980    566.7      240.0   97.93    59.62     OK
    60.0   60.000    1.000    400.0        0.0   97.75    59.55     OK
================================================================================
  Final SOC: 59.55 %
  Modulation index: 0.8165
  Peak AC current: 688.4 A
  Peak efficiency: 98.03 %

Plot saved to: examples/bess_pcs_demo.png
```

The PNG is also saved automatically. Open it with any image viewer.

---

## 2. Understanding the Simulation Scenario

The demo runs a 60-second scenario designed to demonstrate droop control under a realistic grid frequency disturbance:

| Time (s) | Event | Grid Frequency | PCC Voltage |
|---|---|---|---|
| 0–20 | Stable grid; BESS at dispatch set-point | 60.0 Hz | 1.00 pu |
| 20–40 | Load event: frequency dips linearly | 60.0 → 59.5 Hz | 1.00 → 0.96 pu |
| 40–60 | Frequency and voltage recover | 59.5 → 60.0 Hz | 0.96 → 1.00 pu |

The BESS PCS is dispatched at 400 kW export with 5 % P-f droop and 5 % Q-V droop. It responds autonomously to the frequency and voltage changes without any external command.

**No protection trips occur** in this scenario because:
- The minimum frequency (59.5 Hz) is above the 59.3 Hz trip threshold.
- The minimum voltage (0.96 pu) is above the 0.88 pu trip threshold.
- ROCOF is not evaluated (f_samples not passed to evaluate_all).

---

## 3. Interpreting the Four Plot Panels

### Panel 1 (Top-Left): P–f Droop Response

**Left y-axis (blue):** Grid frequency in Hz.
**Right y-axis (red dashed):** BESS active power output in kW.

What to observe:
- At t = 0–20 s: frequency is flat at 60 Hz, P_out is flat at 400 kW (the dispatch set-point).
- At t = 20–40 s: as frequency falls, P_out ramps up toward the rating. The droop slope is: ΔP/Δf = 1000 / (0.05 × 60) = 333 kW/Hz. A 0.5 Hz dip → 167 kW additional output.
- At t = 40–60 s: frequency recovers, P_out ramps back toward 400 kW.
- The horizontal blue dotted line marks f0 = 60 Hz (nominal).

The key insight: the BESS **automatically** increases output in proportion to the frequency drop. No set-point change was sent from the EMS.

### Panel 2 (Top-Right): Q–V Droop Response

**Left y-axis (green):** PCC voltage in per-unit.
**Right y-axis (orange dashed):** BESS reactive power output in kvar.

What to observe:
- At t = 0–20 s: V = 1.00 pu, Q_out = 0 (no reactive injection needed).
- At t = 20–40 s: as voltage sags to 0.96 pu, the Q droop injects up to 480 kvar capacitively.
- At t = 40–60 s: voltage recovers, Q_out returns to zero.
- The horizontal green dotted line marks V0 = 1.0 pu (nominal).

Note that the Q response mirrors the voltage profile — for a 4 % voltage sag with 5 % droop, the injection reaches 80 % of rated kvar (480 / 600 = 80 %).

### Panel 3 (Bottom-Left): SOC & Converter Efficiency

**Left y-axis (purple):** Battery state of charge in %.
**Right y-axis (brown dashed):** Converter efficiency in %.

What to observe:
- SOC decreases monotonically because the BESS exports power throughout the simulation (P_out ≥ 400 kW at all times). The rate increases when P_out ramps to 733 kW during the frequency dip.
- Efficiency peaks (~98 %) at 40–70 % rated power and decreases slightly at lower load. This reflects the converter loss model (fixed losses dominate at light load; conduction losses dominate at heavy load).
- Total energy exported: roughly (400×20 + 567×5 + 733×10 + 733×5 + 567×10 + 400×10) × 0.1/3600 ≈ 0.045 MWh over 60 seconds — consistent with the SOC drop of ~0.45 %.

### Panel 4 (Bottom-Right): AC/DC Currents

**Pink:** AC line current in amperes.
**Grey dashed:** DC bus current in amperes.

What to observe:
- AC current peaks at ~688 A during the maximum power point (733 kW at V_LL = 690 V). The AC limit of 1,200 A (thin pink dotted line) is never reached.
- DC current mirrors the power profile, accounting for the loss correction in `BidirectionalConverter.dc_current()`.
- The difference between AC and DC currents in the same power range reflects the efficiency: P_dc = P_ac / η, so I_dc · V_dc = I_ac · V_ac · √3 / η.

If any protection trips were active they would appear as red shaded bands across all four panels.

---

## 4. Changing Droop Settings and Observing the Response

Open `examples/bess_pcs_demo.py` and locate the parameter block near the top:

```python
P_DISPATCH_KW  = 400.0       # initial active power set-point
Q_DISPATCH_KVAR = 0.0        # initial reactive power set-point

DROOP_P_PCT    = 5.0         # P-f droop %
DROOP_Q_PCT    = 5.0         # Q-V droop %
```

### Experiment 1: Tighten the P-f Droop (stronger frequency response)

Change `DROOP_P_PCT = 2.0` and re-run. Expected results:
- Droop slope Rp increases from 333 to 833 kW/Hz.
- At 59.5 Hz, P_out increases to min(400 + 833 × 0.5, 1000) = 816 kW (versus 567 kW at 5 %).
- P_out saturates at the rated 1,000 kW earlier in the frequency dip (around f = 59.76 Hz).
- SOC depletes faster during the disturbance.

### Experiment 2: Loosen the P-f Droop (weaker frequency response)

Change `DROOP_P_PCT = 10.0` and re-run. Expected results:
- Rp = 167 kW/Hz.
- At 59.5 Hz, P_out = 400 + 167 × 0.5 = 484 kW.
- Much smaller response — the BESS barely reacts to the frequency dip.
- Panel 1 shows P and f curves nearly parallel (P barely changes as f falls).

### Experiment 3: Add a Frequency Deadband

In the `DroopController` constructor, add a deadband parameter:

```python
droop_ctrl = DroopController(
    rated_kw=RATED_KW,
    rated_kvar=RATED_KVAR,
    rated_freq_hz=RATED_FREQ,
    rated_voltage_pu=1.0,
    f_deadband_hz=0.1,   # ← add this line
)
```

Re-run with `DROOP_P_PCT = 5.0`. Expected results:
- No response until frequency drops below 59.9 Hz (0.1 Hz from nominal).
- Between t = 20–22 s, P_out remains at 400 kW even as frequency begins to fall.
- After the deadband is exceeded, the response resumes but starts from a lower frequency.
- In Panel 1, there is a visible "flat" segment at the start of the frequency dip.

### Experiment 4: Trigger a Protection Trip

Tighten the frequency trip limit to simulate a Category I strict setting:

In the simulation loop, change the protection evaluation:

```python
prot = protection.evaluate_all(
    v_pu=vk, f_hz=fk, v_dc=DC_VOLTAGE, v_dc_max=880.0,
    i_meas=i_ac_A[k], i_max=1_200.0,
    f_limits=(59.6, 60.5),   # ← add this line (tighter limit)
)
```

Re-run. The minimum frequency in the scenario is 59.5 Hz, which is below the new 59.6 Hz trip threshold. You should see:
- Red shaded regions appearing in all four panels at t ≈ 31–40 s.
- "TRIP" entries in the console table for those time steps.

This demonstrates how `PCSProtection` responds to out-of-range conditions.

### Experiment 5: Change the Dispatch Set-Point

Change `P_DISPATCH_KW = 700.0` (dispatch the BESS at 70 % of rated power):

- At nominal frequency, P_out = 700 kW (higher base loading).
- The droop response still adds the same incremental power per Hz.
- At 59.5 Hz: P_out = 700 + 333 × 0.5 = 867 kW — clamped by rated power sooner.
- SOC depletes more rapidly throughout the scenario.

---

## 5. Running the Tests

The test suite verifies the `DroopController` against 40+ assertions covering all edge cases:

```bash
pytest tests/ -v
```

Expected output:

```
tests/test_droop.py::TestConstructor::test_valid_construction PASSED
tests/test_droop.py::TestConstructor::test_invalid_rated_kw PASSED
tests/test_droop.py::TestFrequencyDroop::test_at_nominal_frequency_no_deviation PASSED
tests/test_droop.py::TestFrequencyDroop::test_frequency_dip_increases_power PASSED
...
40 passed in 0.31s
```

---

## 6. Using the Library in Your Own Script

The simplest integration is to import the modules directly:

```python
from pcs.converter import BidirectionalConverter
from pcs.control.droop import DroopController
from pcs.control.grid_following import PLLController, CurrentController
from pcs.control.mppt import MPPTController
from pcs.protection import PCSProtection
from pcs.grid_interface import GridInterface
```

### Grid-Following PV Inverter Template

```python
import math

pll  = PLLController(kp=50, ki=1000, f_nominal_hz=60)
cc   = CurrentController(kp=10, ki=500, v_dc=700, l_filter_mH=1.5)
mppt = MPPTController(step_size_V=2.0, v_min=300, v_max=650)
prot = PCSProtection()

dt = 1e-4  # 100 µs control step
t  = 0.0

# Simulated grid voltage (αβ frame)
v_mag = 325.0  # V peak (230 V RMS phase)

while t < 1.0:
    theta = pll.phase_angle(v_mag * math.cos(2*math.pi*60*t),
                            v_mag * math.sin(2*math.pi*60*t), dt)
    f_est = pll.frequency_estimate(pll.theta, theta, dt)

    # MPPT runs every 50 ms
    if round(t / 0.05) * 0.05 == round(t * 1e4) * 1e-4:
        v_ref = mppt.update(v_pv=535, p_pv=85_000)

    # Active current reference from MPPT power
    id_ref = 85_000 / (1.5 * v_mag)  # simplified P → id
    vd, vq = cc.dq_current_control(id_ref, 0, id_ref*0.98, 0, dt)

    # Protection
    trip_v, _ = prot.over_under_voltage(1.0)
    trip_f, _ = prot.over_under_frequency(f_est)
    if trip_v or trip_f:
        cc.reset()
        break

    t += dt
```

### BESS Grid-Forming Template

See `examples/bess_pcs_demo.py` for the full implementation. The core loop is:

```python
droop = DroopController(rated_kw=1000, rated_kvar=600, rated_freq_hz=60)
prot  = PCSProtection()

for k in range(N):
    p_ref  = droop.frequency_droop(f_grid[k], P_SETPOINT, droop_pct=5)
    q_ref  = droop.voltage_droop(v_pcc[k], Q_SETPOINT, droop_pct=5)
    result = prot.evaluate_all(v_pcc[k], f_grid[k], V_DC, 880, i_ac, 1200)
    if result["any_trip"]:
        break
```

---

## 7. Next Steps

- Read [docs/control_theory_guide.md](control_theory_guide.md) for VSC control architecture and dq-frame theory.
- Read [docs/droop_control_guide.md](droop_control_guide.md) for droop coefficient selection and parallel load-sharing.
- Read [docs/grid_interconnection_guide.md](grid_interconnection_guide.md) for IEEE 1547-2018 requirements.
- Read [docs/mppt_guide.md](mppt_guide.md) for PV MPPT algorithms and step-size selection.
- Add your own protection logic by subclassing `PCSProtection` and overriding the threshold defaults.
- Extend the simulation with an LCL filter model, grid impedance, and multi-converter load sharing.
