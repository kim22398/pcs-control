# PCS Control Toolkit

![Python](https://img.shields.io/badge/Python-3.9%2B-blue?logo=python)
![License](https://img.shields.io/badge/License-MIT-green)
![Tests](https://img.shields.io/badge/Tests-pytest-orange)
![Standard](https://img.shields.io/badge/Standard-IEEE%201547--2018-red)
![Topology](https://img.shields.io/badge/Topology-VSC%20%7C%20NPC%20%7C%20T--type-blueviolet)

A professional Python toolkit for Power Conversion System (PCS) control, simulation, and analysis — targeting grid-tied inverters, Battery Energy Storage System (BESS) PCS, and bidirectional AC/DC converters for renewable energy integration.

---

## Table of Contents

1. [Overview](#overview)
2. [Theory & Background](#theory--background)
3. [Architecture](#architecture)
4. [Project Structure](#project-structure)
5. [Quick Start](#quick-start)
6. [API Reference](#api-reference)
7. [Use Cases](#use-cases)
8. [Engineering Standards](#engineering-standards)
9. [Documentation](#documentation)
10. [License](#license)

---

## Overview

Modern grid-connected PCS must satisfy tight requirements for power quality, grid support, protection, and interoperability. This library provides a complete, standards-aligned simulation and control toolkit:

| Module | Description |
|---|---|
| **Converter model** | Efficiency curves, current ratings, modulation index for 3-level NPC / T-type VSC |
| **Grid interface** | Fault level, PCC voltage regulation, islanding non-detection zone (NDZ) |
| **Grid-forming control** | P-f and Q-V droop for autonomous frequency / voltage support in microgrids |
| **Grid-following control** | Synchronous Reference Frame PLL and dq-frame inner current controller |
| **MPPT** | Perturb & Observe algorithm for PV front-ends |
| **Protection** | IEEE 1547-2018 / IEC 62116 compliant OV/UV, OF/UF, ROCOF, DC overvoltage, and overcurrent |

---

## Theory & Background

### Voltage-Source Converter (VSC) Topology

Grid-tied PCS are almost universally implemented as Voltage-Source Converters (VSC). The DC bus is maintained at a controlled voltage (typically 600–1,500 V for utility-scale systems), and an IGBT or SiC MOSFET bridge synthesises a sinusoidal AC output via Sinusoidal PWM (SPWM) or Space-Vector PWM (SVPWM).

The **3-level Neutral-Point-Clamped (NPC)** and **T-type** topologies dominate the multi-MW market because they:
- Reduce device voltage stress to V_dc / 2 per switch.
- Lower output harmonic content, reducing LCL-filter size.
- Achieve peak efficiency of 98–99 % with modern SiC devices.

The AC-side filter (L or LCL) limits the current ripple injected into the grid and is the primary plant for the inner current controller. This toolkit models the converter in the steady-state domain; switching-level dynamics are abstracted through the loss model.

### dq Reference Frame (Park Transform)

The standard approach to controlling a 3-phase VSC is to transform all AC quantities from the stationary abc frame to the synchronously rotating dq frame (Park transform). In the dq frame, sinusoidal quantities become DC values, allowing simple PI controllers to achieve zero steady-state error:

```
[fd]   [cos(θ)    sin(θ)  ] [fα]
[fq] = [-sin(θ)   cos(θ) ] [fβ]
```

where θ is the grid voltage angle tracked by the PLL, and fα, fβ are the Clarke-transformed (αβ) stationary-frame quantities.

- **d-axis** is aligned with the grid voltage vector → controls active current / active power.
- **q-axis** is orthogonal → controls reactive current / reactive power.

The dq transformation is the foundation of the `PLLController` and `CurrentController` classes.

### Grid-Forming vs. Grid-Following

| Feature | Grid-Following | Grid-Forming |
|---|---|---|
| Synchronisation | Requires external grid (PLL) | Self-synchronises (sets angle/frequency) |
| Control variable | Current reference (id, iq) | Voltage/frequency reference |
| Island capability | No — trips at islanding | Yes — can supply islanded load |
| Load-sharing | Via communication or central dispatch | Autonomous via droop |
| Representative class | `PLLController` + `CurrentController` | `DroopController` |
| Application | Grid-tied solar, wind | BESS, virtual synchronous generator, microgrid |

A **grid-forming** inverter behaves like a controlled voltage source and is essential for microgrids and systems that must black-start or support weak grids. A **grid-following** inverter is a controlled current source and requires a stiff voltage reference to lock onto.

### IEEE 1547-2018 Interconnection Requirements

IEEE 1547-2018 is the foundational U.S. standard for distributed energy resource (DER) interconnection. Key requirements implemented in `PCSProtection`:

- **Voltage ride-through**: Continuous operation between 0.88–1.10 pu; Category I/II/III define trip delays for abnormal voltages.
- **Frequency ride-through**: Continuous operation 59.3–60.5 Hz; ROCOF ≤ 2 Hz/s per interconnection agreement.
- **Reactive power capability**: DER ≥ 500 kVA must provide reactive power range of ±0.44 pu of rated current.
- **Anti-islanding**: Passive (ROCOF, OV/UV) and active injection methods must clear island within 2 s.
- **Reconnection**: After a trip, the DER must not reconnect for a minimum delay and must verify stable grid conditions.

See [docs/grid_interconnection_guide.md](docs/grid_interconnection_guide.md) for a full requirement-by-requirement mapping.

---

## Architecture

The control hierarchy follows the standard cascade structure for grid-forming BESS PCS:

```
┌───────────────────────────────────────────────────────────────────┐
│                      ENERGY MANAGEMENT SYSTEM                     │
│              (dispatch set-points P*, Q*, SOC targets)            │
└────────────────────────────┬──────────────────────────────────────┘
                             │ P_setpoint, Q_setpoint
                             ▼
┌───────────────────────────────────────────────────────────────────┐
│              OUTER LOOP: DROOP CONTROLLER                         │
│   P-f droop:  P_ref = P* + Rp·(f0 − f_meas)                     │
│   Q-V droop:  Q_ref = Q* + Rq·(V0 − V_meas)                     │
│   Class: DroopController                                          │
└────────────────────────────┬──────────────────────────────────────┘
                             │ id_ref, iq_ref (via P/Q → I conversion)
                             ▼
┌───────────────────────────────────────────────────────────────────┐
│         INNER LOOP: dq CURRENT CONTROLLER                         │
│   PI with cross-coupling decoupling and voltage feed-forward      │
│   vd_ref = kp·(id_ref−id) + ki·∫(id_ref−id)dt − ω·L·iq + vd_ff │
│   vq_ref = kp·(iq_ref−iq) + ki·∫(iq_ref−iq)dt + ω·L·id + vq_ff │
│   Class: CurrentController                                        │
└────────────────────────────┬──────────────────────────────────────┘
                             │ vd_ref, vq_ref (modulation reference)
                             ▼
┌───────────────────────────────────────────────────────────────────┐
│               MODULATOR (SVPWM / SPWM)                           │
│   Inverse Park → Clarke → Gate signals                            │
│   Modulation index m_a computed by BidirectionalConverter         │
└────────────────────────────┬──────────────────────────────────────┘
                             │ Gate pulses → VSC bridge
                             ▼
┌───────────────────────────────────────────────────────────────────┐
│   PROTECTION (supervisory, parallel to all loops)                 │
│   OV/UV · OF/UF · ROCOF · DC OV · Overcurrent                    │
│   Class: PCSProtection                                            │
└───────────────────────────────────────────────────────────────────┘
```

For **grid-following** mode the outer loop is replaced by an SRF-PLL (`PLLController`) that measures the grid angle, and current references are derived directly from the power set-point dispatched by the EMS.

For **PV applications** an additional MPPT outer loop (`MPPTController`) precedes the droop / current controller and provides the DC voltage reference to maximize PV harvest.

---

## Project Structure

```
pcs-control/
│
├── pcs/                            # Main package
│   ├── __init__.py                 # Package metadata and public API exports
│   ├── converter.py                # BidirectionalConverter — VSC steady-state model
│   │                               #   (efficiency curve, DC/AC current, modulation index)
│   ├── grid_interface.py           # GridInterface — PCC analysis
│   │                               #   (fault level, voltage rise, islanding NDZ)
│   ├── protection.py               # PCSProtection — IEEE 1547-2018 trip logic
│   │                               #   (OV/UV, OF/UF, ROCOF, DC OV, overcurrent)
│   │
│   └── control/                    # Control algorithms sub-package
│       ├── __init__.py             # Control sub-package exports
│       ├── droop.py                # DroopController — P-f and Q-V droop (grid-forming)
│       ├── grid_following.py       # PLLController + CurrentController (grid-following)
│       └── mppt.py                 # MPPTController — Perturb & Observe MPPT
│
├── examples/
│   ├── bess_pcs_demo.py            # 1 MW / 2 MWh BESS droop simulation (60 s)
│   └── bess_pcs_demo.png           # Auto-generated 4-panel results figure
│
├── tests/
│   └── test_droop.py               # pytest suite — DroopController (40+ assertions)
│
├── docs/
│   ├── control_theory_guide.md     # VSC control architecture, Park transform, PLL, PI design
│   ├── droop_control_guide.md      # P-f / Q-V droop theory, numerical example
│   ├── grid_interconnection_guide.md  # IEEE 1547-2018 requirements, ride-through tables
│   ├── mppt_guide.md               # PV MPPT algorithms (P&O, InC, fixed-voltage)
│   └── getting_started.md          # Tutorial: run the demo, interpret plots, change settings
│
├── requirements.txt                # numpy, matplotlib (minimal dependencies)
└── README.md                       # This file
```

---

## Quick Start

### Prerequisites

- Python 3.9 or later
- `pip install -r requirements.txt` (numpy, matplotlib)

### Install and run the demo

```bash
git clone https://github.com/<your-username>/pcs-control.git
cd pcs-control
pip install -r requirements.txt
python examples/bess_pcs_demo.py
```

The demo simulates a 1 MW / 2 MWh BESS with P-f and Q-V droop over a 60-second grid-frequency-dip event and prints a tabular summary:

```
================================================================================
  BESS PCS Droop Simulation — 1 MW / 2 MWh
================================================================================
   t (s)   f (Hz)   V (pu)   P (kW)   Q (kvar)   η (%)  SOC (%)   Trip
--------------------------------------------------------------------------------
     0.0   60.000    1.000    400.0        0.0   97.75    60.00     OK
    20.0   60.000    1.000    400.0        0.0   97.75    59.93     OK
    25.0   59.875    0.990    567.4      240.0   97.93    59.89     OK
    40.0   59.500    0.960    733.3      480.0   98.03    59.82     OK
    60.0   60.000    1.000    400.0        0.0   97.75    59.73     OK
================================================================================
```

A four-panel PNG is saved to `examples/bess_pcs_demo.png` — see [docs/getting_started.md](docs/getting_started.md) for a full walkthrough.

### Minimal usage example

```python
from pcs.converter import BidirectionalConverter
from pcs.control.droop import DroopController
from pcs.protection import PCSProtection

# 1 MW converter on a 690 V / 800 V DC bus
conv  = BidirectionalConverter(rated_kw=1000, dc_voltage_V=800, ac_voltage_V=690)
droop = DroopController(rated_kw=1000, rated_kvar=600, rated_freq_hz=60)
prot  = PCSProtection()

# Droop response at 59.7 Hz, 0.97 pu voltage
p_kw   = droop.frequency_droop(f_measured=59.7, p_setpoint=400, droop_pct=5)
q_kvar = droop.voltage_droop(v_measured=0.97,   q_setpoint=0,   droop_pct=5)

# Converter metrics
eta   = conv.efficiency(p_kw / 1000)
i_dc  = conv.dc_current(p_kw)
i_ac  = conv.ac_current_A(p_kw, pf=0.95)

# Protection check
trip, reason = prot.over_under_voltage(v_pu=0.97)
print(f"P={p_kw:.1f} kW  Q={q_kvar:.1f} kvar  η={eta*100:.2f}%  {reason}")
```

---

## API Reference

### `pcs.converter.BidirectionalConverter`

Steady-state model of a bidirectional three-phase VSC. The loss model is calibrated to a 3-level NPC topology with SiC MOSFETs (~98 % peak efficiency).

```python
BidirectionalConverter(
    rated_kw: float,           # Rated active power [kW]
    dc_voltage_V: float,       # Nominal DC-link voltage [V]
    ac_voltage_V: float,       # Nominal AC line-to-line RMS voltage [V]
    switching_freq_hz: float = 10_000,  # PWM switching frequency [Hz]
)
```

| Method | Signature | Returns | Description |
|---|---|---|---|
| `efficiency` | `(load_factor: float) → float` | η ∈ (0, 1] | Converter efficiency at given fraction of rated power |
| `dc_current` | `(power_kw: float) → float` | A | DC-link current including loss correction |
| `ac_current_A` | `(power_kw: float, pf: float = 1.0) → float` | A (RMS) | AC line current for 3-phase balanced operation |
| `modulation_index` | `() → float` | m_a | SPWM modulation index; >1 indicates over-modulation |

**Example:**
```python
conv = BidirectionalConverter(rated_kw=1000, dc_voltage_V=800, ac_voltage_V=690)
print(conv.efficiency(0.75))       # ~0.979
print(conv.dc_current(750))        # ~963 A
print(conv.ac_current_A(750, 0.95))  # ~661 A
print(conv.modulation_index())     # ~0.865
```

---

### `pcs.grid_interface.GridInterface`

Point-of-common-coupling (PCC) analysis: fault current, voltage regulation, and islanding non-detection zone.

```python
GridInterface(
    grid_voltage_kv: float,       # PCC line-to-line voltage [kV]
    grid_freq_hz: float,          # Nominal frequency [Hz] (50 or 60)
    short_circuit_mva: float,     # Three-phase fault level at PCC [MVA]
    xr_ratio: float = 10.0,       # Source impedance X/R ratio
)
```

| Method | Signature | Returns | Description |
|---|---|---|---|
| `fault_level_kA` | `() → float` | kA (RMS) | Symmetrical three-phase fault current at PCC |
| `pcc_voltage_pu` | `(injected_kvar, injected_kw=0) → float` | pu | Steady-state PCC voltage from Thevenin ΔV formula |
| `islanding_detection_ndz` | `(...) → dict` | dict | Non-detection zone per IEC 62116 Annex B |
| `z_source_ohm` | property | complex Ω | Thevenin source impedance including X/R ratio |

**Example:**
```python
gi = GridInterface(grid_voltage_kv=11, grid_freq_hz=60, short_circuit_mva=150)
print(gi.fault_level_kA())                  # 7.87 kA
print(gi.pcc_voltage_pu(injected_kvar=500)) # ~1.003 pu
ndz = gi.islanding_detection_ndz()
print(ndz["ndz_area_pu2"])                  # non-detection zone area
```

---

### `pcs.control.droop.DroopController`

Stateless P-f and Q-V droop controller for grid-forming PCS. Implements load-sharing characteristics analogous to synchronous generator governor (P-f) and automatic voltage regulator (Q-V).

```python
DroopController(
    rated_kw: float,              # Rated active power [kW]
    rated_kvar: float,            # Rated reactive power [kvar]
    rated_freq_hz: float = 60.0,  # Nominal frequency [Hz]
    rated_voltage_pu: float = 1.0,# Nominal voltage [pu]
    f_deadband_hz: float = 0.0,   # Frequency deadband ±[Hz]
    v_deadband_pu: float = 0.0,   # Voltage deadband ±[pu]
)
```

| Method | Signature | Returns | Description |
|---|---|---|---|
| `frequency_droop` | `(f_measured, p_setpoint, droop_pct=5.0) → float` | kW | P-f droop: P_out = P* + Rp·(f0 − f_meas), clamped to [0, rated_kw] |
| `voltage_droop` | `(v_measured, q_setpoint, droop_pct=5.0) → float` | kvar | Q-V droop: Q_out = Q* + Rq·(V0 − V_meas), clamped to ±rated_kvar |
| `operating_point` | `(f_measured, v_measured, p_setpoint, q_setpoint, ...) → dict` | dict | Combined P and Q operating point with percentage load |

**Droop characteristic equations:**

```
P-f:  Rp = P_rated / (droop_pct/100 · f0)   [kW/Hz]
      P_out = P* − Rp · (f_meas − f0)

Q-V:  Rq = Q_rated / (droop_pct/100 · V0)   [kvar/pu]
      Q_out = Q* − Rq · (V_meas − V0)
```

**Example:**
```python
dc = DroopController(rated_kw=1000, rated_kvar=600, rated_freq_hz=60)
# Frequency dip to 59.7 Hz with 5 % droop → governor response
p = dc.frequency_droop(f_measured=59.7, p_setpoint=400, droop_pct=5)
# → P increases above 400 kW to arrest frequency decline
print(f"P = {p:.1f} kW")   # 500.0 kW
```

---

### `pcs.control.grid_following.PLLController`

Synchronous Reference Frame PLL (SRF-PLL) for grid synchronisation. Tracks the grid voltage angle by aligning the d-axis with the grid voltage vector and driving vq → 0 via a PI compensator.

```python
PLLController(
    kp: float = 50.0,             # Proportional gain [rad/s per V]
    ki: float = 1_000.0,          # Integral gain [rad/s² per V]
    f_nominal_hz: float = 60.0,   # Feed-forward frequency [Hz]
)
```

| Method | Signature | Returns | Description |
|---|---|---|---|
| `phase_angle` | `(v_alpha, v_beta, dt) → float` | rad | Update PLL state, return θ ∈ [0, 2π) |
| `frequency_estimate` | `(theta_prev, theta_curr, dt) → float` | Hz | Instantaneous frequency from consecutive angle samples (handles 2π wrap) |
| `reset` | `(theta_init=0.0)` | — | Reset PLL state (e.g. after fault) |
| `theta` | property | rad | Current angle estimate |

**SRF-PLL error signal:**
```
vq = −vα·sin(θ) + vβ·cos(θ)   ← q-axis projection; → 0 at lock
ω  = ω_ff + kp·vq + ki·∫vq dt
θ  = ∫ω dt  (mod 2π)
```

**Example:**
```python
import math
pll = PLLController(kp=50, ki=1000, f_nominal_hz=60)
# Simulate 10 ms at grid angle 30°
v_mag = 325.0  # V peak
theta_true = 2 * math.pi * 60 * 0.01  # angle after 10 ms
v_alpha = v_mag * math.cos(theta_true)
v_beta  = v_mag * math.sin(theta_true)
theta_est = pll.phase_angle(v_alpha, v_beta, dt=1e-4)
```

---

### `pcs.control.grid_following.CurrentController`

PI inner current controller in the synchronous (dq) rotating frame with cross-coupling decoupling and voltage feed-forward. Implements conditional integration anti-windup.

```python
CurrentController(
    kp: float = 10.0,             # Proportional gain [V/A]
    ki: float = 500.0,            # Integral gain [V/(A·s)]
    v_dc: float = 800.0,          # DC-link voltage [V] (output clamp)
    l_filter_mH: float = 1.0,     # AC-side filter inductance [mH]
    omega_grid: float | None = None,  # Grid angular frequency [rad/s]
    i_max: float = 2_000.0,       # Peak current limit for anti-windup [A]
)
```

| Method | Signature | Returns | Description |
|---|---|---|---|
| `dq_current_control` | `(id_ref, iq_ref, id_meas, iq_meas, dt, vd_ff=0, vq_ff=0) → (float, float)` | (vd_ref, vq_ref) [V] | One-step PI update with decoupling and anti-windup |
| `reset` | `()` | — | Reset integrator states |

**Controller equations:**
```
vd_ref = kp·(id_ref−id) + ki·∫(id_ref−id)dt + vd_ff − ω·L·iq
vq_ref = kp·(iq_ref−iq) + ki·∫(iq_ref−iq)dt + vq_ff + ω·L·id
```

Output is clamped to the space-vector limit: |v_dq| ≤ V_dc / √3.

---

### `pcs.control.mppt.MPPTController`

Perturb & Observe (P&O) MPPT controller for PV front-ends. Returns the voltage reference fed to the DC/DC stage or the outer PCS control loop.

```python
MPPTController(
    step_size_V: float = 1.0,    # Perturbation step ΔV [V]
    v_min: float = 200.0,        # Minimum voltage reference [V]
    v_max: float = 800.0,        # Maximum voltage reference [V]
    v_init: float | None = None, # Initial reference (defaults to midpoint)
)
```

| Method | Signature | Returns | Description |
|---|---|---|---|
| `update` | `(v_pv: float, p_pv: float) → float` | V | Process one sample; return new V_ref |
| `track` | `(v_pv_list, p_pv_list) → list[dict]` | list | Offline batch analysis over historical data |
| `reset` | `()` | — | Reset to initial conditions |
| `v_ref` | property | V | Current voltage reference |

**P&O decision table:**
```
ΔP > 0 and ΔV ≥ 0  →  V_ref += ΔV   (keep moving right toward MPP)
ΔP > 0 and ΔV < 0  →  V_ref −= ΔV   (keep moving left toward MPP)
ΔP < 0 and ΔV ≥ 0  →  V_ref −= ΔV   (reverse: move left)
ΔP < 0 and ΔV < 0  →  V_ref += ΔV   (reverse: move right)
```

---

### `pcs.protection.PCSProtection`

IEEE 1547-2018 / IEC 62116 protection functions. All instantaneous methods are stateless — they evaluate a single measurement against thresholds. All methods return `(trip: bool, reason: str)`.

```python
PCSProtection()   # stateless; thresholds passed per-call
```

| Method | Default thresholds | Description |
|---|---|---|
| `over_under_voltage(v_pu, limits=(0.88, 1.10))` | 0.88–1.10 pu (Cat A) | OV/UV per IEEE 1547-2018 Table 3 |
| `over_under_frequency(f_hz, limits=(59.3, 60.5))` | 59.3–60.5 Hz | OF/UF per IEEE 1547-2018 Table 4 |
| `rate_of_change_of_frequency(f_samples, dt, threshold=1.0)` | 1.0 Hz/s | ROCOF from first-difference derivative |
| `dc_overvoltage(v_dc, v_max)` | caller-specified | DC-link overvoltage (e.g. load rejection) |
| `current_limit(i_measured, i_max)` | caller-specified | AC overcurrent / hardware limit |
| `evaluate_all(v_pu, f_hz, v_dc, v_dc_max, i_meas, i_max, ...)` | combined defaults | Evaluate all protections; returns summary dict |

**Example:**
```python
prot = PCSProtection()

trip, reason = prot.over_under_voltage(v_pu=1.12)
# → (True, 'OV: 1.1200 pu > 1.1 pu')

trip, reason = prot.over_under_frequency(f_hz=59.1)
# → (True, 'UF: 59.1000 Hz < 59.3 Hz')

# ROCOF from 5 consecutive frequency measurements
f_samples = [60.0, 59.8, 59.5, 59.1, 58.8]
trip, reason = prot.rate_of_change_of_frequency(f_samples, dt=0.02, threshold=1.0)
# → (True, 'ROCOF: ...')

# Full evaluation
results = prot.evaluate_all(
    v_pu=1.0, f_hz=60.0, v_dc=800, v_dc_max=880,
    i_meas=900, i_max=1200
)
print(results["any_trip"])  # False
```

---

## Use Cases

### 1. BESS Grid-Forming PCS

A battery energy storage system operating in grid-forming mode autonomously regulates island frequency and voltage using droop control. The `DroopController` responds to frequency deviations without communication, dispatching additional power during under-frequency events and absorbing power during over-frequency events — analogous to a synchronous generator governor.

```python
# 1 MW / 600 kVAr BESS PCS with 4 % droop
droop = DroopController(rated_kw=1000, rated_kvar=600, rated_freq_hz=60,
                        f_deadband_hz=0.02)  # 20 mHz deadband

# Island frequency dips to 59.6 Hz — BESS ramps to ~800 kW
p = droop.frequency_droop(59.6, p_setpoint=400, droop_pct=4)
```

See `examples/bess_pcs_demo.py` for a full 60-second transient simulation.

### 2. PV Solar Inverter

A grid-following solar inverter uses the MPPT controller to maximise PV harvest and the SRF-PLL for grid synchronisation. The current controller injects the harvested power at unity power factor (iq_ref = 0).

```python
mppt = MPPTController(step_size_V=2.0, v_min=300, v_max=700)
pll  = PLLController(kp=50, ki=1000)
cc   = CurrentController(kp=10, ki=500, v_dc=700)

# MPPT provides DC voltage reference → determines id_ref from P_pv
v_ref = mppt.update(v_pv=540, p_pv=95_000)
```

### 3. STATCOM / Reactive Power Compensator

A static synchronous compensator (STATCOM) operates with zero active power (id_ref = 0) and uses the Q-V droop to provide fast reactive power support. The `CurrentController` in pure q-axis mode achieves sub-cycle reactive current injection.

```python
droop = DroopController(rated_kw=1, rated_kvar=1000, rated_freq_hz=60)
q_ref = droop.voltage_droop(v_measured=0.94, q_setpoint=0, droop_pct=3)
# → 933 kvar capacitive injection at 6 % voltage sag
```

### 4. Virtual Synchronous Generator (VSG)

A VSG emulates inertia by adding a swing-equation integrator around the droop controller. The `DroopController` provides the steady-state droop characteristic; a virtual inertia constant H (in seconds) is emulated by integrating the frequency error:

```
J_virtual · dω/dt = P_mechanical − P_electrical − D·Δω
```

where P_mechanical comes from the droop characteristic and D is a damping coefficient. This is the foundation of grid-forming converters that can replace synchronous generation.

---

## Engineering Standards

| Standard | Scope | Coverage in this Toolkit |
|---|---|---|
| **IEEE 1547-2018** | DER interconnection and interoperability | OV/UV, OF/UF, ROCOF, anti-islanding, reactive power capability — all implemented in `PCSProtection` |
| **IEC 62116:2014** | Islanding prevention test procedure | NDZ calculation in `GridInterface.islanding_detection_ndz()`; trip logic in `PCSProtection` |
| **IEC 61727:2004** | PV systems — grid interface characteristics | Voltage/frequency trip windows; compatible with `PCSProtection` defaults |
| **UL 1741** | Inverters and converters for distributed energy | Superseded by UL 1741-SA (supplement A) aligning with IEEE 1547-2018; protection thresholds match |
| **IEEE 519-2014** | Harmonic current limits | THD limits noted; not computed (switching-level model required) |
| **NERC PRC-024-3** | Generator frequency/voltage relay settings | Frequency ride-through reference for high-penetration scenarios |
| **CIGRE WG C6.22** | Microgrid engineering and economics | Droop control design recommendations used in `DroopController` |

---

## Documentation

| Document | Description |
|---|---|
| [docs/control_theory_guide.md](docs/control_theory_guide.md) | VSC control architecture, Park transform, SRF-PLL design, inner current PI tuning, virtual inertia |
| [docs/droop_control_guide.md](docs/droop_control_guide.md) | P-f / Q-V droop theory, coefficient selection, deadband design, parallel converter load-sharing |
| [docs/grid_interconnection_guide.md](docs/grid_interconnection_guide.md) | IEEE 1547-2018 requirements: ride-through categories, ROCOF, anti-islanding, reconnection |
| [docs/mppt_guide.md](docs/mppt_guide.md) | PV MPPT algorithms: P&O, incremental conductance, step-size trade-offs, partial shading |
| [docs/getting_started.md](docs/getting_started.md) | Tutorial: run the BESS demo, interpret the four plot panels, change droop settings |

---

## License

MIT — see [LICENSE](LICENSE) for details.
