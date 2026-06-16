# Droop Control Guide

A detailed engineering reference for P-f and Q-V droop control as implemented in `DroopController` — covering the underlying theory, droop coefficient selection, deadband design, dynamic response, parallel converter load sharing, and a worked numerical example.

---

## 1. What is Droop Control?

Droop control is a decentralised frequency and voltage regulation strategy for grid-forming inverters. It mimics the behaviour of a synchronous generator with a speed governor (for frequency / active power) and an automatic voltage regulator (for voltage / reactive power), enabling multiple converters to share load autonomously without a communication link.

### 1.1 The Synchronous Generator Analogy

A synchronous generator with a governor exhibits the following steady-state characteristic: as the shaft mechanical power increases, the governor throttle opens, but the speed settles slightly below the no-load setpoint — this is the **droop** or **speed regulation** characteristic. The slope is intentionally set so that parallel generators share load in proportion to their ratings.

Grid-forming inverters replicate this by intentionally allowing frequency and voltage to deviate slightly from nominal as a function of output power. This deviation carries information about the loading state and allows autonomous load sharing.

---

## 2. P-f Droop (Active Power – Frequency)

### 2.1 Characteristic Equation

The P-f droop characteristic defines a linear relationship between frequency deviation and active power output:

```
P_out  =  P*  +  Rp · (f0 − f_meas)

where:
    P*     = active power set-point [kW]  (EMS dispatch target at f0)
    f0     = nominal frequency [Hz]        (60 Hz or 50 Hz)
    f_meas = measured / estimated frequency [Hz]
    Rp     = P-f droop slope [kW/Hz]
```

The droop slope is parameterised by the droop percentage:

```
Rp  =  P_rated / (droop_pct/100 · f0)    [kW/Hz]
```

A **5 % droop** means that a full-rated frequency deviation (5 % of 60 Hz = 3 Hz) produces a power change equal to the full rated power of the converter. In practice the converter never reaches 5 % frequency deviation; the characteristic simply sets the slope.

### 2.2 Sign Convention

- Frequency below f0 (under-frequency event) → P_out increases above P* → converter injects more power → supports frequency recovery.
- Frequency above f0 (over-frequency event, e.g. load rejection) → P_out decreases below P* → converter absorbs more power → prevents over-frequency.

This is the **governor response** convention: the converter acts as a frequency-responsive load / generator.

### 2.3 Droop Slope Interpretation

| Droop % | Rp (1 MW rated, 60 Hz) | Power change at ±0.5 Hz deviation |
|---|---|---|
| 2 % | 833 kW/Hz | ±417 kW |
| 4 % | 417 kW/Hz | ±208 kW |
| 5 % | 333 kW/Hz | ±167 kW |
| 8 % | 208 kW/Hz | ±104 kW |
| 10 % | 167 kW/Hz | ±83 kW |

Tighter droop (lower %) = stronger frequency support but less equal load sharing between parallel converters.

---

## 3. Q-V Droop (Reactive Power – Voltage)

### 3.1 Characteristic Equation

The Q-V droop characteristic defines a linear relationship between voltage deviation and reactive power output:

```
Q_out  =  Q*  +  Rq · (V0 − V_meas)

where:
    Q*     = reactive power set-point [kvar]
    V0     = nominal voltage [pu]             (typically 1.0)
    V_meas = measured PCC voltage [pu]
    Rq     = Q-V droop slope [kvar/pu]
```

```
Rq  =  Q_rated / (droop_pct/100 · V0)    [kvar/pu]
```

### 3.2 Sign Convention

- Voltage below V0 (voltage sag) → Q_out positive → converter injects capacitive reactive power → raises PCC voltage.
- Voltage above V0 (voltage surge) → Q_out negative → converter absorbs inductive reactive power → lowers PCC voltage.

This matches standard SVC / STATCOM convention: positive Q = capacitive injection.

### 3.3 Q-V Droop Slope Selection

For voltage control applications the droop slope directly determines the steady-state voltage regulation:

```
ΔV  =  ΔQ / Rq  (approximately, for stiff grid)
```

A tighter Q-V droop provides better voltage regulation but makes parallel converters more sensitive to small voltage differences, potentially causing reactive power circulation. A 5 % droop is a common default.

---

## 4. Droop Coefficient Selection

### 4.1 P-f Droop Selection Criteria

1. **Grid code requirements**: Many grid codes specify a droop range of 2–12 % for frequency-responsive generation. IEEE 1547-2018 allows droop setting by interconnection agreement.
2. **System inertia**: In low-inertia systems (high inverter penetration), tighter droop (2–4 %) provides faster frequency containment.
3. **Load-sharing requirements**: If N parallel converters of equal rating should share load equally, they should all have the same droop percentage (not the same droop slope). If units have different ratings (P1, P2), equal droop % gives load sharing proportional to rating.
4. **Battery SoC management**: For BESS, droop setpoints can be adjusted as a function of SoC. At high SoC: tighter droop to export more; at low SoC: looser droop or reduced P_setpoint.

### 4.2 Q-V Droop Selection Criteria

1. **Voltage regulation band**: With 5 % Q-V droop, the steady-state voltage variation at full reactive power load is 5 % of V0 = 0.05 pu. This fits within the IEEE 1547-2018 continuous operating range of 0.88–1.10 pu.
2. **Reactive power circulation**: In parallel converters, even small voltage measurement offsets cause steady-state reactive current circulation. Droop > 3 % helps dampen this without communication.
3. **Feeder X/R ratio**: On resistive LV feeders (X/R < 1), P has more influence on voltage than Q. An alternative "inverse droop" (P-V, Q-f) is sometimes used in LV microgrids.

---

## 5. Deadband Design

### 5.1 Purpose

A deadband is a zero-response region around the nominal operating point. Within the deadband, small frequency or voltage deviations do not trigger droop action.

**Reasons to use a deadband:**
- Prevents continuous power oscillation due to measurement noise.
- Avoids wear on battery systems (unnecessary micro-cycles).
- Aligns with utility requirements (some interconnections specify no response within ±0.036 Hz).

### 5.2 Deadband Implementation

The `DroopController` applies deadband by zeroing the deviation if it falls within the band:

```python
delta_f = f_measured - f0
if abs(delta_f) <= f_deadband_hz:
    delta_f = 0.0
P_out = P_setpoint - Rp * delta_f
```

This creates a discontinuity at the deadband edge. For smoother response, a **soft deadband** can be implemented by linearly interpolating the response over a transition zone, but the hard deadband is standard practice and is what `DroopController` implements.

### 5.3 Typical Deadband Values

| Application | Frequency deadband | Voltage deadband |
|---|---|---|
| BESS frequency response | ±0.02–0.05 Hz | — |
| Microgrid island control | 0 (no deadband) | 0.01 pu |
| Virtual power plant | ±0.05 Hz | — |
| Primary frequency response | ±0.015–0.03 Hz | — |

---

## 6. Static vs. Dynamic Response

### 6.1 Static (Steady-State) Response

The droop equations describe the **static characteristic** — the steady-state operating point after all transients have settled. The `DroopController` is a pure static mapping; it produces an instantaneous output for any input frequency / voltage.

The static response is what determines:
- Frequency nadir (lowest frequency after a generation loss event).
- Steady-state voltage regulation band.
- Load-sharing accuracy between parallel converters.

### 6.2 Dynamic Response

In a real system the droop characteristic acts as the reference for the inner current controller. The **dynamic response** (how fast the output tracks the droop reference) is determined by:

1. **Rate limiting**: Many implementations add a rate-of-change limit (ramp rate) on the droop output to avoid mechanical stress on grid transformers or battery current spikes. Typical: 10–20 % of rated power per second.
2. **Inertia emulation**: A virtual inertia integrator adds a J·dω/dt term, slowing the response intentionally to emulate synchronous generator inertia.
3. **Inner loop bandwidth**: The current controller must be at least 5–10× faster than the droop outer loop. With a droop settling time of ~100 ms, the current controller bandwidth should exceed 50 rad/s (~8 Hz).

### 6.3 Small-Signal Stability

For a single converter connected to an infinite bus through impedance Z, the droop controller is unconditionally stable. With multiple parallel converters the droop slope acts as a damping term, and stability can be analysed using small-signal state-space methods. The key stability requirement is that the product Rp × (grid impedance) does not create a right-half-plane zero in the closed-loop transfer function.

---

## 7. Parallel Converter Load Sharing

### 7.1 Equal Load Sharing

Consider N parallel grid-forming converters with ratings P_i and droop percentages d_i. In island mode all converters share a common frequency f. The droop characteristic for converter i is:

```
P_i  =  P*_i  +  Rp_i · (f0 − f)
```

The total power matches total load:

```
P_load  =  Σ P_i  =  Σ P*_i  +  (Σ Rp_i) · (f0 − f)
```

The frequency settles at:

```
f  =  f0  −  (P_load − Σ P*_i) / Σ Rp_i
```

Each converter takes a share proportional to its droop slope Rp_i. If all converters have the **same droop percentage** d %, then:

```
Rp_i  =  P_rated_i / (d/100 · f0)
```

and the load is shared proportionally to the rated power of each converter — which is the desired outcome.

### 7.2 Load Sharing Accuracy

In practice, load sharing accuracy is limited by:
- Voltage measurement offsets (Q-V droop especially sensitive).
- Frequency measurement errors (relevant if different PLLs see different noise).
- Set-point (P*, Q*) differences caused by EMS communication latency.

A tighter droop reduces the sensitivity to set-point errors at the cost of larger steady-state frequency / voltage deviation.

---

## 8. Worked Numerical Example

### System Parameters

- Two BESS PCS units operating in parallel island mode.
- Unit 1: P_rated = 1,000 kW, d_p = 5 %, P* = 400 kW.
- Unit 2: P_rated = 500 kW, d_p = 5 %, P* = 200 kW.
- Nominal frequency: f0 = 60 Hz.
- Island load: 900 kW (constant).

### Step 1: Droop Slopes

```
Rp1  =  1000 / (0.05 × 60)  =  333.3 kW/Hz
Rp2  =   500 / (0.05 × 60)  =  166.7 kW/Hz
```

### Step 2: Steady-State Frequency

Total set-point: P*_total = 600 kW.
Load: 900 kW.
Power deficit: 300 kW.

```
f  =  f0  −  ΔP / (Rp1 + Rp2)
   =  60  −  300 / (333.3 + 166.7)
   =  60  −  300 / 500
   =  60  −  0.6  =  59.4 Hz
```

### Step 3: Individual Converter Outputs

```
P1  =  400  +  333.3 × (60 − 59.4)  =  400 + 200  =  600 kW
P2  =  200  +  166.7 × (60 − 59.4)  =  200 + 100  =  300 kW
```

Total: 600 + 300 = 900 kW ✓

### Step 4: Load-Sharing Check

```
P1 / P_rated_1  =  600 / 1000  =  60 %
P2 / P_rated_2  =  300 /  500  =  60 %
```

Both units are loaded to the same percentage — equal load sharing achieved with the same droop percentage.

### Step 5: Reproducing with DroopController

```python
from pcs.control.droop import DroopController

unit1 = DroopController(rated_kw=1000, rated_kvar=600, rated_freq_hz=60)
unit2 = DroopController(rated_kw=500,  rated_kvar=300, rated_freq_hz=60)

f_island = 59.4  # settled island frequency

p1 = unit1.frequency_droop(f_island, p_setpoint=400, droop_pct=5)
p2 = unit2.frequency_droop(f_island, p_setpoint=200, droop_pct=5)

print(f"P1 = {p1:.1f} kW  ({100*p1/1000:.0f} % of rated)")
print(f"P2 = {p2:.1f} kW  ({100*p2/500:.0f} % of rated)")
print(f"Total = {p1+p2:.1f} kW")
# P1 = 600.0 kW  (60 % of rated)
# P2 = 300.0 kW  (60 % of rated)
# Total = 900.0 kW
```

---

## 9. Extensions and Variants

### 9.1 Secondary Frequency Restoration

Pure droop control leaves a steady-state frequency error (the 0.6 Hz deviation in the example above). A **secondary control layer** (SCADA or microgrid central controller) periodically adjusts the P* set-points of all converters to restore frequency to exactly f0 without changing the droop slopes.

In the example: after secondary control acts, P*1 = 600 kW, P*2 = 300 kW, and f returns to 60 Hz.

### 9.2 Adaptive Droop

The droop coefficient can be varied dynamically based on:
- Battery SoC (loosen droop as SoC approaches minimum).
- Grid frequency trend (tighten droop during sustained under-frequency events).
- Temperature (derate droop slope to limit current at high temperatures).

### 9.3 Non-Linear Droop

A piecewise-linear or polynomial droop characteristic can be used to provide stronger response near the extremes of the operating range while remaining insensitive to small frequency deviations in the centre. Some grid codes (e.g. ENTSO-E) specify deadbands with non-linear extensions.

---

## References

1. Guerrero et al. "Hierarchical Control of Droop-Controlled AC and DC Microgrids." *IEEE Trans. Ind. Electron.*, vol. 58, no. 1, 2011.
2. CIGRE WG C6.22. *Microgrids 1: Engineering, Economics & Experience.* 2015.
3. IEEE 1547.4-2011. *Guide for Design, Operation, and Integration of Distributed Resource Island Systems.*
4. Bevrani, Ise, Miura. "Virtual Synchronous Generators: A Survey and New Perspectives." *Int. J. Elect. Power Energy Syst.*, 2014.
5. Schiffer et al. "Conditions for Stability of Droop-Controlled Inverter-Based Microgrids." *Automatica*, 2014.
