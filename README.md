# PCS Control Toolkit

A Python toolkit for Power Conversion System (PCS) control, simulation, and analysis — targeting grid-tied inverters, Battery Energy Storage System (BESS) PCS, and bidirectional AC/DC converters for renewable energy integration.

---

## Overview

Modern grid-connected PCS must satisfy tight requirements for power quality, grid support, and protection. This library provides:

- **Converter modelling** — efficiency curves, current ratings, modulation index for voltage-source inverters
- **Grid interface analysis** — fault level, PCC voltage regulation, islanding non-detection zone
- **Grid-forming control** — P-f and Q-V droop for autonomous frequency/voltage support
- **Grid-following control** — Phase-Locked Loop (PLL) and dq-frame current controller
- **MPPT** — Perturb & Observe algorithm for PV front-ends
- **Protection** — IEEE 1547 / IEC 62116 compliant voltage, frequency, ROCOF, and DC overvoltage trips

### Target applications

| Application | Key features used |
|---|---|
| BESS grid-forming PCS | Droop control, protection |
| PV + storage inverter | MPPT, grid-following, protection |
| Grid-tied bidirectional converter | Converter model, grid interface, current controller |
| Microgrid PCS | Droop, islanding detection |

---

## Repository structure

```
pcs-control/
├── pcs/
│   ├── converter.py          # BidirectionalConverter model
│   ├── grid_interface.py     # GridInterface — fault level, PCC voltage, islanding NDZ
│   ├── protection.py         # PCSProtection — trip logic for all standard protections
│   └── control/
│       ├── droop.py          # DroopController — P-f and Q-V droop (grid-forming)
│       ├── grid_following.py # PLLController + CurrentController (grid-following)
│       └── mppt.py           # MPPTController — Perturb & Observe
├── examples/
│   └── bess_pcs_demo.py      # 1 MW / 2 MWh BESS droop simulation
├── tests/
│   └── test_droop.py         # pytest suite for droop controller
├── requirements.txt
└── README.md
```

---

## Quick start

```bash
pip install -r requirements.txt
python examples/bess_pcs_demo.py
```

---

## Module reference

### `pcs.converter.BidirectionalConverter`

```python
conv = BidirectionalConverter(
    rated_kw=1000,
    dc_voltage_V=800,
    ac_voltage_V=400,
    switching_freq_hz=10_000,
)
eta  = conv.efficiency(load_factor=0.75)   # 0–1
i_dc = conv.dc_current(power_kw=750)       # A
i_ac = conv.ac_current_A(power_kw=750, pf=0.95)  # A (line, 3-phase)
mi   = conv.modulation_index()             # SPWM index
```

Efficiency is modelled with fixed + proportional losses (conduction + switching), calibrated to a typical 3-level NPC topology (~98 % peak).

### `pcs.grid_interface.GridInterface`

```python
gi = GridInterface(grid_voltage_kv=11, grid_freq_hz=60, short_circuit_mva=150)
ikA  = gi.fault_level_kA()                          # three-phase fault current
v_pu = gi.pcc_voltage_pu(injected_kvar=500)         # voltage rise at PCC
ndz  = gi.islanding_detection_ndz()                 # non-detection zone dict
```

### `pcs.control.droop.DroopController`

Implements standard P-f and Q-V droop per IEEE 1547.4 / CIGRE recommendations.

```python
dc = DroopController(rated_kw=1000, rated_kvar=600,
                     rated_freq_hz=60, rated_voltage_pu=1.0)
p_out = dc.frequency_droop(f_measured=59.8, p_setpoint=500, droop_pct=5)
q_out = dc.voltage_droop(v_measured=0.97, q_setpoint=0, droop_pct=5)
```

### `pcs.control.grid_following`

```python
pll = PLLController(kp=50, ki=1000)
theta = pll.phase_angle(v_alpha, v_beta, dt=1e-4)
f_est = pll.frequency_estimate(theta_prev, theta_curr, dt=1e-4)

cc = CurrentController(kp=10, ki=500, v_dc=800)
vd_ref, vq_ref = cc.dq_current_control(
    id_ref=100, iq_ref=0, id_meas=98, iq_meas=2, dt=1e-4
)
```

### `pcs.control.mppt.MPPTController`

```python
mppt = MPPTController(step_size_V=1.0, v_min=200, v_max=800)
v_ref = mppt.update(v_pv=540, p_pv=95_000)   # call each sample period
history = mppt.track(v_pv_list, p_pv_list)   # offline analysis
```

### `pcs.protection.PCSProtection`

All methods return `(trip: bool, reason: str)`.

```python
prot = PCSProtection()
prot.over_under_voltage(v_pu=1.12)            # (True, "OV")
prot.over_under_frequency(f_hz=58.9)          # (False, "OK")
prot.rate_of_change_of_frequency(f_samples, dt=0.02, threshold=1.0)
prot.dc_overvoltage(v_dc=870, v_max=850)
prot.current_limit(i_measured=1250, i_max=1200)
```

---

## Theory notes

### Droop control

The P-f characteristic relates active power to frequency:

```
f = f0 - Rp * (P - P0)    where Rp = droop_pct / 100 * f0 / P_rated
```

The Q-V characteristic:

```
V = V0 - Rq * (Q - Q0)    where Rq = droop_pct / 100 * V_rated / Q_rated
```

### PLL (Synchronous Reference Frame)

The SRF-PLL tracks the grid angle θ by driving vq → 0:

```
ε = vq = |v| sin(θ_grid - θ_pll)
θ̇ = ωff + kp·ε + ki·∫ε dt
```

### ROCOF islanding detection

ROCOF is estimated from consecutive frequency measurements:

```
df/dt = (f[k] - f[k-1]) / dt
```

A trip is issued when |df/dt| > threshold (typical 0.5–2 Hz/s per IEEE 1547-2018).

---

## Standards compliance

| Standard | Coverage |
|---|---|
| IEEE 1547-2018 | Voltage/frequency trip windows, ROCOF, islanding |
| IEC 62116:2014 | Islanding non-detection zone |
| IEEE 519-2014 | (THD limits noted; not computed) |
| NERC PRC-024 | Frequency ride-through reference |

---

## License

MIT
