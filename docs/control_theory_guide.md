# VSC Control Theory Guide

A technical reference for the control architecture implemented in the PCS Control Toolkit — covering converter topology, the dq reference frame transformation, SRF-PLL operation, inner current controller design, outer power/voltage control, and the distinction between grid-forming and grid-following converters.

---

## 1. Voltage-Source Converter Topology

### 1.1 Basic Structure

The Voltage-Source Converter (VSC) is the dominant power electronic interface for grid-connected applications. Its defining characteristic is a stiff DC voltage source (typically a large capacitor bank) on one side and a controllable AC voltage synthesiser (the bridge) on the other.

For a three-phase, two-level VSC the bridge consists of six switches (IGBT or SiC MOSFET) arranged in three legs. Each leg produces a switched voltage waveform between 0 and V_dc. After LC or LCL filtering the fundamental component is a clean sinusoid. The switching frequency (typically 4–20 kHz for SiC devices) determines the harmonic spectrum and the bandwidth of the current controller.

### 1.2 3-Level NPC / T-Type Topology

Utility-scale PCS (500 kW – 5 MW) predominantly use **3-level Neutral-Point-Clamped (NPC)** or **T-type** topologies. In a 3-level NPC each leg produces three voltage levels: +V_dc/2, 0, −V_dc/2. The benefits over two-level:

- Each switch only blocks V_dc/2 → lower device voltage ratings.
- Output voltage has lower harmonic content (lowest significant harmonic is at 2×f_sw rather than f_sw).
- Filter size is halved for the same current ripple specification.
- Peak efficiency 98–99 % achievable with SiC MOSFETs.

The `BidirectionalConverter` class models this topology in the steady state: the efficiency curve uses loss coefficients calibrated to a modern SiC NPC module, and the modulation index is computed assuming sinusoidal PWM (SPWM) with a linear modulation range of m_a ≤ 1.

### 1.3 AC-Side Filter

The filter between the converter output and the grid serves two purposes:

1. **Current ripple rejection** — the switching harmonics (at n×f_sw ± m×f_grid) must not exceed IEEE 519-2014 THD limits at the PCC.
2. **Control plant** — the filter inductance L is the dominant plant for the inner current controller.

For a simple L-filter the plant transfer function is:

```
I(s) / V(s)  =  1 / (L·s + R)
```

where R is the winding resistance. An LCL filter provides better high-frequency attenuation but introduces a resonant pole that must be damped (passively or actively) and complicates the current controller design.

---

## 2. The dq Reference Frame (Park Transform)

### 2.1 Motivation

In the stationary abc frame, all currents and voltages are sinusoidal at grid frequency. A PI controller tracking a sinusoidal reference in steady state has a finite gain at the fundamental frequency, leading to a non-zero steady-state error. The solution is to transform all quantities into a **synchronously rotating reference frame** where they appear as DC values — then a PI controller achieves zero steady-state error.

### 2.2 Clarke Transform (abc → αβ)

The first step is the Clarke transform, which reduces three-phase (abc) quantities to two-phase stationary-frame (αβ) quantities. For a balanced three-phase system:

```
[fα]   [1    -1/2    -1/2  ] [fa]
[fβ] = [0   √3/2   -√3/2  ] [fb]
                              [fc]
```

(with the appropriate scaling factor — amplitude-invariant convention uses 2/3; power-invariant uses √(2/3).)

The αβ frame is stationary. The grid voltage vector rotates at angular frequency ω = 2πf, tracing a circle in the αβ plane.

### 2.3 Park Transform (αβ → dq)

The Park transform projects the αβ quantities onto a frame rotating at the same angular frequency as the grid voltage vector:

```
[fd]   [ cos(θ)   sin(θ)] [fα]
[fq] = [-sin(θ)   cos(θ)] [fβ]
```

where θ = ∫ω dt is the instantaneous grid angle tracked by the PLL.

In a correctly aligned dq frame:
- **d-axis** is locked to the grid voltage phasor → V_d = |V|, V_q = 0.
- Active power is proportional to I_d; reactive power is proportional to I_q.
- All quantities are DC in steady state.

The `PLLController` maintains the tracking of θ so that the Park transform remains aligned despite grid frequency variations and small voltage disturbances.

### 2.4 Inverse Transform for Modulation

After the current controller computes voltage references V_d_ref and V_q_ref in the dq frame, they must be inverse-transformed back to abc for the modulator:

```
[Vα]   [cos(θ)  -sin(θ)] [Vd]
[Vβ] = [sin(θ)   cos(θ)] [Vq]
```

Then the Clarke inverse gives V_a, V_b, V_c, which are the references for SPWM comparison with the triangular carrier.

---

## 3. Synchronous Reference Frame PLL (SRF-PLL)

### 3.1 Operating Principle

The SRF-PLL is the standard grid synchronisation method for grid-following converters. The goal is to find the angle θ that makes the Park-transformed q-axis voltage zero:

```
Vq = −Vα·sin(θ_pll) + Vβ·cos(θ_pll)
```

When θ_pll = θ_grid: Vq = |V|·sin(0) = 0. For small angle errors, Vq ≈ |V|·(θ_grid − θ_pll), making it a linear error signal.

The `PLLController.phase_angle()` method implements this directly — Vq is computed from the αβ inputs and the current angle estimate, then fed through a PI compensator:

```
ω_out = ω_ff + kp·Vq + ki·∫Vq dt
θ     = ∫ω_out dt  (mod 2π)
```

The feed-forward term ω_ff = 2π·f_nominal ensures the PLL starts near the correct frequency and reduces the burden on the integral term.

### 3.2 PI Tuning Guidelines

The PLL bandwidth is determined by the PI gains and the grid voltage magnitude |V| (which acts as the plant gain). A typical design approach:

1. Choose PLL bandwidth ω_n (rad/s). Values of 50–500 rad/s are common; lower bandwidth → more noise rejection, higher bandwidth → faster transient response.
2. Set kp = 2·ζ·ω_n / |V|, where ζ is the damping ratio (typically 0.707).
3. Set ki = ω_n² / |V|.

For |V| = 325 V (peak phase voltage at 230 V RMS), ω_n = 100 rad/s, ζ = 0.707:
- kp ≈ 0.44
- ki ≈ 30.8

The `PLLController` defaults (kp=50, ki=1000) are calibrated for per-unit voltages. When using SI volts, adjust accordingly.

### 3.3 Frequency Estimation

The `PLLController.frequency_estimate()` method derives instantaneous frequency from consecutive angle samples:

```
Δθ = θ[k] − θ[k-1]   (unwrapped to (−π, π])
f  = Δθ / (2π·dt)
```

This frequency estimate is used by the ROCOF protection function and also for damping in virtual synchronous generator implementations.

---

## 4. Inner Current Controller Design

### 4.1 Plant Model in the dq Frame

The AC-side L-filter dynamics in the dq frame are coupled by the cross terms introduced by the rotating frame transformation:

```
L · d(Id)/dt  =  Vd_conv − Vd_grid − R·Id + ω·L·Iq
L · d(Iq)/dt  =  Vq_conv − Vq_grid − R·Iq − ω·L·Id
```

The ±ω·L·I cross-coupling terms cause the d and q axes to interact. If not compensated, a step in I_d reference will also perturb I_q and vice versa.

### 4.2 Decoupling and Feed-Forward

The `CurrentController` applies the standard decoupling structure:

```
Vd_ref = PI_d(Id_ref − Id) + Vd_ff − ω·L·Iq   ← removes q→d coupling
Vq_ref = PI_q(Iq_ref − Iq) + Vq_ff + ω·L·Id   ← removes d→q coupling
```

With perfect decoupling the d and q axes are independent first-order systems:

```
I(s) / I_ref(s)  =  (kp·s + ki) / (L·s² + (R + kp)·s + ki)
```

### 4.3 PI Tuning (Internal Model Control Method)

The Internal Model Control (IMC) method gives a systematic closed-form tuning:

1. Choose closed-loop bandwidth α_c (rad/s). Typical values: 500–2000 rad/s for a 10 kHz switching converter.
2. Set kp = α_c · L
3. Set ki = α_c · R

For L = 1 mH, R = 0.05 Ω, α_c = 1000 rad/s:
- kp = 1.0 V/A
- ki = 50 V/(A·s)

The `CurrentController` defaults (kp=10, ki=500) reflect a normalised design; adjust for the actual L-filter parameters using `l_filter_mH`.

### 4.4 Anti-Windup

When the voltage output is saturated (|V_dq| > V_dc/√3), the integrator continues accumulating error and causes integrator windup — the controller takes excessive time to recover after the saturation event ends.

The `CurrentController` implements **conditional integration** anti-windup: the integrator is frozen on an axis when its raw (unsaturated) output magnitude exceeds V_dc/2. This simple scheme is effective for current controllers where saturation events are infrequent.

---

## 5. Outer Power / Voltage Control

### 5.1 Power Reference Generation

In grid-following mode the outer loop receives active (P*) and reactive (Q*) power set-points from the EMS and converts them to dq current references:

```
Id_ref = (2/3) · P* / Vd_grid
Iq_ref = −(2/3) · Q* / Vd_grid
```

(using amplitude-invariant convention with Vd_grid = |V|, Vq_grid = 0.)

These references are passed to the `CurrentController` at each control step.

### 5.2 Droop Outer Loop (Grid-Forming)

In grid-forming mode the `DroopController` replaces the EMS dispatch with an autonomous droop characteristic:

```
P_ref = P* − Rp · (f_meas − f0)
Q_ref = Q* − Rq · (V_meas − V0)
```

This is the primary outer loop. The P and Q references are then converted to Id/Iq references for the current controller, or directly used as the amplitude and phase set-points for a voltage reference generator in a voltage-mode outer loop.

The droop gain Rp and Rq set the trade-off between frequency / voltage regulation accuracy and load-sharing:
- Small droop (2 %) → tight frequency regulation, unequal load sharing.
- Large droop (10 %) → equal load sharing, larger steady-state frequency deviation.

### 5.3 Voltage-Mode Grid-Forming

An alternative outer loop generates a voltage reference directly (voltage-mode control), without an explicit current controller inner loop. The output voltage of the VSC is controlled by a voltage PI loop, and current limiting is achieved by saturation. This mode is more suitable for microgrid black-start but requires careful stability analysis due to the reduced inner loop bandwidth.

---

## 6. Grid-Forming vs. Grid-Following

### 6.1 Classification

**Grid-following (GFL)** converters:
- Operate as controlled current sources.
- Use a PLL to synchronise with the grid angle.
- Cannot operate in isolation (no grid → PLL loses lock).
- The majority of today's DER fleet (wind, solar) is GFL.

**Grid-forming (GFM)** converters:
- Operate as controlled voltage sources.
- Establish their own angle and frequency reference internally (droop, VSG).
- Can supply islanded loads and support black start.
- Required for high-inverter-penetration systems.

### 6.2 Stability Considerations

GFL converters can interact adversely with each other and with weak grids. The PLL introduces a phase lag that limits the maximum short-circuit ratio (SCR) at which the converter can operate stably. For SCR < 2 (very weak grids), GFL instability is a known concern.

GFM converters, by establishing their own voltage reference, do not suffer from PLL-related instability. However, their fast voltage control can interact with the network impedance, particularly in stiff grids where the GFM converter may "fight" with the transmission system. Droop-based GFM converters are more robust than bang-bang voltage regulators in this regard.

### 6.3 Virtual Inertia

One significant advantage of synchronous generators (SGs) is their inherent kinetic inertia — the rotational energy stored in the rotor. When grid frequency deviates, the rotor naturally absorbs or releases energy, providing an immediate frequency damping response before any governor action.

A **Virtual Synchronous Generator (VSG)** emulates this behaviour by computing a virtual rotor speed and angle:

```
J · dω/dt  =  Tm − Te − D·(ω − ω0)
θ           =  ∫ω dt
```

where J is the virtual inertia (kg·m²), Tm is the virtual mechanical torque from the droop characteristic, Te is the electrical torque from the measured output power, and D is a damping coefficient.

The `DroopController` provides the steady-state droop characteristic (the governor model). A VSG implementation adds the J·dω/dt term as an integrator around it, giving a finite rate of frequency change instead of the instantaneous response of a pure droop controller.

For a BESS PCS implementing a VSG, a typical inertia constant H = 5 s means that the system can sustain rated power for 5 seconds from kinetic energy alone — emulated by discharging the battery in proportion to the virtual rotor's kinetic energy deficit.

---

## 7. Summary: Control Bandwidth Hierarchy

| Loop | Bandwidth (typical) | Class |
|---|---|---|
| Inner current controller | 500–2000 rad/s (80–320 Hz) | `CurrentController` |
| PLL tracking bandwidth | 50–500 rad/s (8–80 Hz) | `PLLController` |
| Outer droop / power control | 2–20 rad/s (0.3–3 Hz) | `DroopController` |
| EMS / SCADA dispatch | 0.001–0.1 rad/s (set-point updates every 10–60 s) | External |

The cascade structure requires each inner loop to be at least 5–10× faster than the loop above it so that the inner loop appears as unity gain to the outer controller.

---

## References

1. Teodorescu, Liserre, Rodriguez. *Grid Converters for Photovoltaic and Wind Power Systems*. Wiley-IEEE, 2011.
2. Blaabjerg et al. "Overview of Control and Grid Synchronization for Distributed Power Generation Systems." *IEEE Trans. Ind. Electron.*, 2006.
3. Guerrero et al. "Hierarchical Control of Droop-Controlled AC and DC Microgrids." *IEEE Trans. Ind. Electron.*, 2011.
4. D'Arco, Suul. "Equivalence of Virtual Synchronous Machines and Frequency-Droops for Converter-Based Microgrids." *IEEE Trans. Smart Grid*, 2014.
5. Zmood, Holmes. "Stationary Frame Current Regulation of PWM Inverters with Zero Steady-State Error." *IEEE Trans. Power Electron.*, 2003.
