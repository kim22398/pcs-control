# Grid Interconnection Guide — IEEE 1547-2018

A comprehensive reference for the IEEE 1547-2018 interconnection requirements applicable to grid-tied PCS, with a requirement-by-requirement mapping to the `PCSProtection` class.

---

## 1. Overview of IEEE 1547-2018

IEEE Std 1547-2018 (revision of 1547-2003) is the foundational U.S. standard for the interconnection and interoperability of Distributed Energy Resources (DER) with associated electric power system interfaces. It is adopted by reference in most U.S. utility interconnection tariffs and state regulations.

**Key changes from the 2003 version:**

- Introduction of three DER performance **Categories** (I, II, III) with different ride-through requirements.
- Mandatory **reactive power capability** for DER ≥ 500 kVA.
- Expanded **voltage and frequency ride-through** tables with defined trip clearing times.
- Addition of **ROCOF** (Rate-of-Change of Frequency) requirements for anti-islanding.
- Interoperability requirements including communication interfaces (IEEE 2030.5, SunSpec Modbus).

This toolkit implements the protection and monitoring functions of §7 (Abnormal Operating Performance) and §8 (Interconnection system response to abnormal conditions).

---

## 2. DER Categories

IEEE 1547-2018 defines three performance categories based on the DER's required ride-through capability:

| Category | Description | Typical Applications |
|---|---|---|
| **Category I** | Minimum ride-through requirement; may trip at the limits | Small DER, existing installations |
| **Category II** | Enhanced ride-through; must remain connected through most grid disturbances | New utility-scale DER ≥ 500 kVA |
| **Category III** | Maximum ride-through; must remain connected through severe disturbances | DER critical to grid stability; required by utility |

The category is assigned by the utility in the interconnection agreement. `PCSProtection` defaults match Category A (the legacy designation from the 2003 standard, roughly equivalent to Category I continuous operating band).

---

## 3. Voltage Ride-Through Requirements

### 3.1 Continuous Operating Range

For a 120/240 V system, the continuous operating voltage range at the PCC is 0.88–1.10 pu. DER must remain online indefinitely within this range.

For a 120 V system:
```
V_low_continuous  = 0.88 pu  =  105.6 V
V_high_continuous = 1.10 pu  =  132.0 V
```

### 3.2 Voltage Ride-Through Windows (60 Hz, Categories I / II / III)

IEEE 1547-2018 Table 3 defines the trip time as a function of voltage for each category. The table below gives the clearing time (maximum time before trip) for selected voltage levels:

| Voltage Range (pu) | Category I | Category II | Category III |
|---|---|---|---|
| V < 0.45 | 0.16 s | 0.16 s | 0.16 s (momentary cessation allowed) |
| 0.45 ≤ V < 0.60 | 0.16 s | 0.45 s | 0.45 s |
| 0.60 ≤ V < 0.88 | 2.0 s | 2.0 s | 10.0 s (must ride through) |
| 0.88 ≤ V ≤ 1.10 | Continuous | Continuous | Continuous |
| 1.10 < V ≤ 1.20 | 1.0 s | 1.0 s | 13.0 s |
| V > 1.20 | 0.16 s | 0.16 s | 0.16 s |

**Note:** "Momentary cessation" (temporarily stopping current injection but remaining connected) is allowed for V < 0.45 pu in all categories — the DER does not have to trip, but it may reduce output to zero.

### 3.3 PCSProtection Implementation

`PCSProtection.over_under_voltage()` evaluates the instantaneous voltage against configurable limits:

```python
prot = PCSProtection()
# Category I / default continuous limits
trip, reason = prot.over_under_voltage(v_pu=0.86)
# → (True, 'UV: 0.8600 pu < 0.88 pu')

# Category III extended ride-through — set wider limits
trip, reason = prot.over_under_voltage(v_pu=0.86, limits=(0.45, 1.20))
# → (False, 'OK')  — ride through this level per Cat III
```

The timing logic (hold-off timers for Category II/III) should be implemented in the calling layer; `PCSProtection` provides the instantaneous trip evaluation.

---

## 4. Frequency Ride-Through Requirements

### 4.1 Continuous Operating Range

DER must operate continuously between **59.3 Hz and 60.5 Hz** (for 60 Hz systems).

### 4.2 Frequency Ride-Through Table (60 Hz System)

| Frequency Range (Hz) | Category I | Category II | Category III |
|---|---|---|---|
| f < 57.0 | 0.16 s | 0.16 s | 0.16 s |
| 57.0 ≤ f < 58.5 | 0.16 s | 0.16 s | 0.16 s |
| 58.5 ≤ f < 59.3 | 0.16 s | 0.16 s | 299 s |
| 59.3 ≤ f ≤ 60.5 | Continuous | Continuous | Continuous |
| 60.5 < f ≤ 61.5 | 0.16 s | 0.16 s | 299 s |
| f > 61.5 | 0.16 s | 0.16 s | 0.16 s |

For **50 Hz systems** (IEC 61727 / IEC 62116), the corresponding ranges shift to 47.5–50.2 Hz continuous, with similar proportional margins.

### 4.3 PCSProtection Implementation

```python
# Standard Category I limits (60 Hz)
trip, reason = prot.over_under_frequency(f_hz=59.1)
# → (True, 'UF: 59.1000 Hz < 59.3 Hz')

# Category III extended hold-off at 59.0 Hz — implement timer outside
trip, reason = prot.over_under_frequency(f_hz=59.0, limits=(58.5, 61.5))
# → (False, 'OK') — within Category III continuous range
```

---

## 5. Rate-of-Change of Frequency (ROCOF)

### 5.1 Purpose

ROCOF is the primary active anti-islanding method for DER. When the main grid disconnects, the island frequency rapidly changes due to the power mismatch between local generation and load. ROCOF detectors can identify this characteristic rate of change and trip the DER before the voltage and frequency themselves exceed the OV/UV / OF/UF trip windows.

### 5.2 IEEE 1547-2018 Requirements

IEEE 1547-2018 §7.6 requires DER to have anti-islanding capability compliant with the standard. The ROCOF threshold is specified in the interconnection agreement, typically:

- 0.5 Hz/s for sensitive interconnections (weak grids, microgrids).
- 1.0 Hz/s for most utility interconnections.
- 2.0 Hz/s for low-impedance / stiff systems.
- Up to 3.0 Hz/s is allowed where the utility can demonstrate that ROCOF > 3 Hz/s will not occur from non-islanding events.

### 5.3 ROCOF Estimation

The `PCSProtection.rate_of_change_of_frequency()` method estimates ROCOF from consecutive frequency samples using the first-difference approximation:

```
df/dt[k]  =  (f[k] − f[k-1]) / dt
```

The peak absolute ROCOF across all sample pairs is compared to the threshold. This method is equivalent to a rectangular window differentiator.

**Example — detecting a 50 MW generation loss on a 1,000 MW island:**

```python
# Frequency samples at 20 ms intervals after a generation trip
f_samples = [60.00, 59.96, 59.88, 59.76, 59.60, 59.42]  # Hz
# True ROCOF ≈ (59.42-60.00)/(5×0.02) = -5.8 Hz/s

trip, reason = prot.rate_of_change_of_frequency(
    f_samples=f_samples, dt=0.02, threshold=1.0
)
# → (True, 'ROCOF: −29.00 Hz/s exceeds ±1.0 Hz/s')
```

### 5.4 ROCOF Limitations

ROCOF-based anti-islanding has known failure modes:

1. **Non-detection zone**: If the power mismatch is very small (ΔP/P_load < 5 %), the ROCOF may not exceed the threshold within the required 2-second clearing window.
2. **False trips during grid disturbances**: A large grid event (major load switching, transmission line trip) can cause ROCOF > threshold without true islanding. Setting the threshold too low increases false trip probability.
3. **Measurement noise**: ROCOF estimation is sensitive to frequency measurement noise. Low-pass filtering the frequency signal before differentiating is standard practice.

The `GridInterface.islanding_detection_ndz()` method quantifies the non-detection zone where passive methods (including ROCOF) may fail.

---

## 6. Reactive Power Capability

### 6.1 IEEE 1547-2018 §7.7 Requirements

DER with nameplate apparent power ≥ 500 kVA **must** be capable of providing:

- A reactive power range of at least ±0.44 pu of rated current (approximately ±44 % of rated apparent power in kVAr).
- Reactive power injection or absorption on demand from the utility EMS or a local volt/var optimisation function.

DER < 500 kVA are exempt from mandatory reactive power capability but must not actively degrade PCC voltage.

### 6.2 Reactive Power Modes

IEEE 1547-2018 defines four standard reactive power control modes (§7.7.2):

| Mode | Description | Implementation |
|---|---|---|
| Constant power factor | Fixed PF at all active power levels | `CurrentController` with fixed iq/id ratio |
| Voltage-reactive power (volt/var) | Q varies with measured voltage | `DroopController.voltage_droop()` |
| Active power-reactive power | Q varies with active power output | Custom droop using P as input |
| Constant reactive power | Fixed Q regardless of conditions | `DroopController` with v_deadband = large |

The `DroopController.voltage_droop()` directly implements the volt/var mode — the Q-V characteristic described in the droop guide is the same as the IEEE 1547-2018 volt/var function.

---

## 7. Anti-Islanding Requirements

### 7.1 Definition

An **unintentional island** forms when a DER continues to energise a portion of the distribution network after the utility grid has disconnected. This is hazardous because:

- The island voltage and frequency may drift outside normal ranges.
- Utility personnel may assume the line is de-energised when it is not.
- Re-closing of the grid into an out-of-phase island can damage equipment and DER.

### 7.2 Passive Anti-Islanding Methods

Passive methods detect islanding by monitoring voltage and frequency at the PCC. When the grid disconnects, the island frequency and voltage drift until they exceed the OV/UV / OF/UF trip windows.

| Method | Implementation | Effectiveness |
|---|---|---|
| OV/UV | `PCSProtection.over_under_voltage()` | Limited — fails when ΔP ≈ 0 |
| OF/UF | `PCSProtection.over_under_frequency()` | Limited — fails when ΔP ≈ 0 |
| ROCOF | `PCSProtection.rate_of_change_of_frequency()` | Better — detects rapid frequency change |
| Vector surge | Rate of change of voltage angle | Not implemented (requires synchronised reference) |

### 7.3 Active Anti-Islanding Methods

Active methods deliberately perturb the converter output to provoke a detectable response in an island. They are required when the passive methods alone cannot reduce the non-detection zone to zero:

- **Reactive power export perturbation**: Inject small pulsed Q steps; in island, V changes; in grid-connected, V stays stable.
- **Frequency shift (slip mode frequency shift)**: Continuously nudge the output frequency; in island, ROCOF accelerates.
- **Sandia frequency shift**: Accelerate the frequency perturbation based on measured frequency deviation.

Active methods are not implemented in this toolkit as they require real-time feedback control. The `GridInterface.islanding_detection_ndz()` method quantifies the residual NDZ from passive methods alone.

### 7.4 Clearing Time

IEEE 1547-2018 §7.7.1 requires the DER to cease energising the island within **2 seconds** of islanding detection under any loading condition. Most utilities require 0.5–1 second in practice.

---

## 8. Reconnection Requirements

### 8.1 IEEE 1547-2018 §7.8

After a protective trip, the DER must not reconnect to the grid until:

1. **Grid voltage** is within the continuous operating range (0.88–1.10 pu) for a minimum observation period.
2. **Grid frequency** is within the continuous operating range (59.3–60.5 Hz).
3. **Reconnection delay** has elapsed:
   - Minimum 20 seconds after voltage and frequency have returned to normal.
   - For Category III (strongly-interconnected systems), the reconnection may be controlled by the utility EMS.

### 8.2 Reconnection Ramp

To avoid a step change in power injection when reconnecting (which can cause voltage and frequency transients), IEEE 1547-2018 recommends a ramp rate not exceeding **the greater of 10 % of rated power per second or 10 kW/s**.

The `DroopController` set-point ramping is the appropriate place to implement this; set P_setpoint to 0 at reconnection and ramp it toward the dispatch target at the required rate.

---

## 9. IEC Standards Comparison

| Requirement | IEEE 1547-2018 (NA) | IEC 61727:2004 (PV, global) | IEC 62116:2014 (anti-islanding) |
|---|---|---|---|
| Under-voltage trip | 0.88 pu / 2 s | 0.85 pu / 0.1 s | Per local standard |
| Over-voltage trip | 1.10 pu / 1 s | 1.10 pu / 0.1 s | Per local standard |
| Under-frequency trip | 59.3 Hz / continuous | 49.5 Hz (50 Hz sys) | Per local standard |
| Over-frequency trip | 60.5 Hz / continuous | 50.5 Hz (50 Hz sys) | Per local standard |
| Anti-islanding | ROCOF + OV/UV + active | OV/UV + active | Standardised test procedure |
| NDZ requirement | NDZ must be zero with combined methods | Same | Test verifies NDZ = 0 |

The `PCSProtection` thresholds are configurable to match either IEEE or IEC requirements.

---

## 10. Summary: PCSProtection Function-to-Standard Mapping

| `PCSProtection` Method | IEEE 1547-2018 Section | IEC Reference |
|---|---|---|
| `over_under_voltage()` | §7.4, Table 3 | IEC 61727 §5.2 |
| `over_under_frequency()` | §7.5, Table 4 | IEC 61727 §5.3 |
| `rate_of_change_of_frequency()` | §7.6 (anti-islanding) | IEC 62116 Annex A |
| `dc_overvoltage()` | §8.1 (equipment protection) | — |
| `current_limit()` | §8.2 (overcurrent protection) | — |
| `evaluate_all()` | Combined evaluation | — |
| `GridInterface.islanding_detection_ndz()` | §7.6 (NDZ analysis) | IEC 62116 Annex B |

---

## References

1. IEEE Std 1547-2018. *Standard for Interconnection and Interoperability of Distributed Energy Resources with Associated Electric Power Systems Interfaces.* IEEE, 2018.
2. IEC 62116:2014. *Test Procedure of Islanding Prevention Measures for Utility-Interconnected Photovoltaic Inverters.* IEC, 2014.
3. IEC 61727:2004. *Photovoltaic (PV) Systems — Characteristics of the Utility Interface.* IEC, 2004.
4. UL 1741-SA. *Standard for Inverters, Converters, Controllers and Interconnection System Equipment for Use With Distributed Energy Resources — Supplement A: Grid Support Utility Interactive Equipment.* UL, 2016.
5. Ropp et al. "Prevention of Islanding in Grid-Connected Photovoltaic Systems." *Progress in Photovoltaics*, 2000.
