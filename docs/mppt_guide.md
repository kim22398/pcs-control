# MPPT Guide — Photovoltaic Maximum Power Point Tracking

A technical reference for PV maximum power point tracking (MPPT) algorithms — covering the PV I-V characteristic, Perturb & Observe (P&O), Incremental Conductance (InC), fixed-voltage method, step-size trade-offs, partial shading behaviour, and how `MPPTController` implements P&O.

---

## 1. The PV Cell I-V Characteristic

### 1.1 Single-Diode Model

A PV cell is modelled by the single-diode equivalent circuit. The output current as a function of terminal voltage is:

```
I  =  Iph  −  I0 · [exp((V + I·Rs) / (n·Vt)) − 1]  −  (V + I·Rs) / Rsh
```

where:
- `Iph` — photocurrent (proportional to irradiance G [W/m²])
- `I0` — dark saturation current (strongly temperature-dependent)
- `Rs`, `Rsh` — series and shunt resistance
- `n` — diode ideality factor (1–2)
- `Vt = kT/q` — thermal voltage (~26 mV at 25 °C)

### 1.2 I-V and P-V Curves

The key features of a PV module's I-V curve at standard test conditions (STC: G = 1000 W/m², T = 25 °C):

| Point | Symbol | Typical value (250 W module) |
|---|---|---|
| Short-circuit current | Isc | 8.5 A |
| Open-circuit voltage | Voc | 37.5 V |
| Maximum power current | Impp | 7.9 A |
| Maximum power voltage | Vmpp | 31.2 V |
| Maximum power | Pmpp | 245 W |
| Fill factor | FF = Pmpp/(Voc·Isc) | ~0.77 |

The P-V curve has a single global maximum at (Vmpp, Pmpp) under uniform irradiance. MPPT algorithms track this point.

### 1.3 Effect of Irradiance and Temperature

- **Irradiance ↑**: Isc increases proportionally; Voc increases logarithmically. Pmpp increases approximately linearly with G.
- **Temperature ↑**: Voc decreases (−0.35 %/°C); Isc increases slightly. Net effect: Pmpp decreases (~−0.4 %/°C).
- Under typical operation, Vmpp varies over a ±15–20 % range compared to its STC value. MPPT must track this variation continuously.

---

## 2. Perturb & Observe (P&O) Algorithm

### 2.1 Principle

P&O is the most widely deployed MPPT algorithm due to its simplicity. The algorithm periodically perturbs the PV voltage reference by a fixed step ΔV and observes the resulting change in output power ΔP.

**Decision table:**

| ΔP | ΔV | Action | Interpretation |
|---|---|---|---|
| > 0 | ≥ 0 | V_ref += ΔV | Moving right toward MPP |
| > 0 | < 0 | V_ref −= ΔV | Moving left toward MPP |
| < 0 | ≥ 0 | V_ref −= ΔV | Overshot; reverse direction |
| < 0 | < 0 | V_ref += ΔV | Overshot; reverse direction |
| = 0 | any | No change | At MPP (rare exact condition) |

In steady state (constant irradiance), the algorithm oscillates between V_mpp − ΔV and V_mpp + ΔV, causing a steady-state power loss equal to approximately:

```
ΔP_loss  ≈  (dP/dV)|_MPP · ΔV  +  (d²P/dV²)|_MPP · ΔV²/2
```

The second derivative term dominates: `ΔP_loss ∝ ΔV²`, so halving the step size reduces steady-state oscillation loss by a factor of 4.

### 2.2 Implementation in `MPPTController`

The `MPPTController.update()` method implements the standard P&O algorithm:

```python
mppt = MPPTController(step_size_V=2.0, v_min=300, v_max=700)

# Simulation loop
for v_pv, p_pv in zip(voltage_samples, power_samples):
    v_ref = mppt.update(v_pv, p_pv)
    # Send v_ref to DC/DC converter or outer PCS loop
```

**First call behaviour**: On the first call, the previous values are seeded and no perturbation is applied. This avoids a spurious decision based on undefined ΔP.

**Voltage limits**: The voltage reference is clamped to [v_min, v_max] at each step, preventing the reference from leaving the useful operating range of the PV array.

### 2.3 Step Size Trade-Off

The choice of step size ΔV is a fundamental design trade-off:

| Large ΔV | Small ΔV |
|---|---|
| Fast convergence to MPP after irradiance change | Slow convergence |
| Large steady-state oscillation loss | Small steady-state loss |
| Robust to noise | Sensitive to measurement noise (false steps) |
| Suitable for rapidly changing irradiance | Suitable for stable irradiance |

**Typical values:**
- Fixed-step P&O: ΔV = 0.5–3 V for a 250 W module (Vmpp ≈ 30 V), corresponding to 1.5–10 % of Vmpp.
- Large arrays (500 V string): ΔV = 2–5 V.

**Quantitative loss estimate:** For a 250 W module at STC, with ΔV = 2 V and (d²P/dV²) ≈ −2.5 W/V² at the MPP:
```
ΔP_loss  ≈  2.5 × 2²/2  =  5 W  (2 % of Pmpp)
```

Reducing to ΔV = 0.5 V reduces steady-state loss to ~0.31 W (0.13 %).

### 2.4 Tracking Efficiency

MPPT **tracking efficiency** is defined as:

```
η_MPPT  =  E_tracked / E_available

where E_available = ∫ P_mpp(t) dt  (energy if always at MPP)
```

For P&O under uniform irradiance: η_MPPT ≈ 98–99.5 % depending on step size and irradiance variability.

Under rapidly varying irradiance (passing clouds): η_MPPT can drop to 90–95 % for standard P&O because the algorithm may momentarily track in the wrong direction when irradiance changes during a perturbation period.

---

## 3. Incremental Conductance (InC) Algorithm

### 3.1 Principle

The Incremental Conductance (InC) algorithm is based on the derivative of the P-V curve at the MPP:

```
dP/dV  =  d(V·I)/dV  =  I + V·dI/dV  =  0  at MPP

→  dI/dV  =  −I/V   at MPP
→  dI/dV  > −I/V   for V < Vmpp  (left of MPP)
→  dI/dV  < −I/V   for V > Vmpp  (right of MPP)
```

Using the approximation `dI/dV ≈ ΔI/ΔV`:

```
If ΔI/ΔV + I/V = 0  → At MPP (stop)
If ΔI/ΔV + I/V > 0  → Left of MPP (increase V)
If ΔI/ΔV + I/V < 0  → Right of MPP (decrease V)
```

### 3.2 Advantages over P&O

- **In theory**: InC can detect the exact MPP and stop perturbing, eliminating steady-state oscillation.
- **In practice**: Measurement noise introduces errors in ΔI/ΔV, and a small oscillation band is still used.
- **Rapidly changing irradiance**: InC is less susceptible to false direction reversals than P&O because it uses the local slope rather than the previous-step power comparison.

### 3.3 Disadvantages

- Requires simultaneous measurement of ΔV and ΔI (two-channel ADC needed).
- More computationally demanding.
- Still vulnerable to partial shading (single local maximum assumption).

InC is not implemented in `MPPTController` — only P&O is provided. The `MPPTController` class can serve as a base class for an InC extension.

---

## 4. Fixed-Voltage Method

### 4.1 Principle

The fixed-voltage (or constant-voltage) MPPT method sets the voltage reference to a fixed fraction of Voc:

```
V_ref  =  k_v · Voc     where  k_v ≈ 0.76
```

This exploits the empirical observation that Vmpp / Voc is approximately constant (0.70–0.82) across different irradiance levels and modules.

### 4.2 Advantages

- Extremely simple to implement (no iterative search needed).
- No measurement noise sensitivity.

### 4.3 Disadvantages

- Requires a Voc measurement, which means momentarily open-circuiting the array (power interruption).
- The ratio k_v varies with temperature (Vmpp drops faster than Voc as T rises).
- Typical tracking efficiency: 92–96 % — significantly lower than P&O or InC.

Fixed-voltage is appropriate for very low-cost systems where a microcontroller with limited computation is used, or as a startup method before P&O takes over.

---

## 5. Partial Shading Problem

### 5.1 Multiple Local Maxima

Under non-uniform irradiance (partial shading from clouds, trees, soiling), bypass diodes in PV modules cause the P-V curve to develop **multiple local maxima**. The bypass diode short-circuits a shaded module group, creating a "step" in the I-V curve.

A 3-module string with one module at 50 % irradiance can exhibit 2–3 local power maxima. Standard P&O and InC will converge to the local maximum nearest to the starting point, which may not be the global maximum.

**Impact:** In severe partial shading, tracking the local rather than global MPP can reduce harvested power by 20–50 % compared to the true maximum.

### 5.2 Global MPPT Approaches

Several methods address the partial shading problem:

1. **Global scan**: Periodically sweep the full voltage range to identify all local maxima, then restart P&O from the global maximum. Cost: power interruption during scan.

2. **Particle Swarm Optimisation (PSO)**: Multiple "particles" (trial operating points) explore the P-V curve simultaneously. Converges to global maximum without full scan but requires more computation.

3. **Fibonacci / Golden-Section search**: Divide-and-conquer search that guarantees global convergence for unimodal functions; extended versions handle multi-peak curves.

4. **Module-level power electronics (MLPE)**: Each module has its own DC/DC converter (microinverter or DC optimizer), eliminating the bypass diode mismatch problem entirely. State of the art for residential PV.

`MPPTController` implements standard P&O without partial shading enhancement. For production systems with partial shading risk, a global scan or PSO wrapper around `MPPTController.track()` should be considered.

### 5.3 Detecting Partial Shading

A practical detection heuristic: if the power-tracking efficiency drops below 95 % for more than 30 seconds (monitored by comparing actual P to an irradiance-normalised model), initiate a global scan. This can be implemented as a supervisory layer around `MPPTController`.

---

## 6. MPPT Control Period Selection

The MPPT control period T_mppt (interval between `update()` calls) should be chosen based on:

1. **Inner loop settling time**: The DC/DC converter or outer PCS voltage control must have settled to the previous V_ref before the next perturbation is applied. Typical: T_mppt ≥ 5× inner loop time constant.
2. **Irradiance variability**: Under clear sky, T_mppt = 50–100 ms is adequate. Under fast-moving clouds, T_mppt = 10–25 ms improves tracking. Very short T_mppt increases sampling noise sensitivity.
3. **Battery life (for PV+BESS)**: Frequent power oscillations cause micro-cycling of the battery. Limiting T_mppt ≥ 20 ms reduces cycling wear.

**Typical practice:**
- PV microinverter: T_mppt = 10–50 ms.
- Central inverter (≥ 100 kW): T_mppt = 50–200 ms.
- `MPPTController` default: caller-controlled; the `update()` method should be called at the selected period.

---

## 7. Integration with PCS Control

### 7.1 DC/DC Stage

In a two-stage PV PCS (PV → DC/DC → DC bus → VSC → Grid), the MPPT controller outputs a V_ref for the DC/DC converter (typically a boost topology). The DC/DC regulates V_pv to V_ref while the VSC independently controls V_dc and P/Q to the grid.

```python
mppt   = MPPTController(step_size_V=2, v_min=300, v_max=600)
droop  = DroopController(rated_kw=250, rated_kvar=100, rated_freq_hz=60)

# MPPT outer loop (every 50 ms)
v_ref_pv = mppt.update(v_pv=540, p_pv=230_000)

# Grid-following inner loop (every 100 µs)
# P_available from MPPT; Q from volt/var
p_ref = p_pv  # follow PV harvest
q_ref = droop.voltage_droop(v_pcc, q_setpoint=0, droop_pct=5)
```

### 7.2 Single-Stage Topology

In a single-stage PV inverter (PV → VSC → Grid), the VSC DC bus is directly connected to the PV array. The MPPT algorithm adjusts the VSC modulation index or the current reference to move the PV operating point.

In this topology the separation between MPPT and current control is tighter, and the step size must account for the AC ripple current reflected onto the DC bus.

### 7.3 Feed-Forward from Irradiance Sensor

For very fast irradiance changes (direct normal irradiance sensor or forecast), a feed-forward term can be added:

```python
# Irradiance-based initial V_ref (avoids cold-start oscillation)
v_ref_ff = 0.76 * v_oc_measured  # fixed-voltage method
# Then P&O refines from this starting point
mppt._v_ref = v_ref_ff
v_ref = mppt.update(v_pv, p_pv)
```

---

## References

1. Esram, Chapman. "Comparison of Photovoltaic Array Maximum Power Point Tracking Techniques." *IEEE Trans. Energy Convers.*, vol. 22, no. 2, 2007.
2. Femia et al. "Optimization of Perturb and Observe MPPT Method." *IEEE Trans. Power Electron.*, vol. 20, no. 4, 2005.
3. Liu, Dougal. "Dynamic Multiphysics Model for Solar Array." *IEEE Trans. Energy Convers.*, 2002.
4. Villalva et al. "Comprehensive Approach to Modeling and Simulation of Photovoltaic Arrays." *IEEE Trans. Power Electron.*, 2009.
5. Tey, Mekhilef. "Modified Incremental Conductance Algorithm for Photovoltaic System Under Partial Shading Conditions and Load Variation." *IEEE Trans. Ind. Electron.*, 2014.
