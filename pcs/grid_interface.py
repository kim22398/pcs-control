"""
pcs.grid_interface
==================
GridInterface — point of common coupling (PCC) analysis for grid-tied PCS.

Covers:
  - Three-phase bolted fault current at the PCC
  - Steady-state voltage rise / drop due to reactive power injection
  - Islanding non-detection zone (NDZ) per IEC 62116 / IEEE 1547

Theory
------
Fault level
    I_fault = S_sc / (√3 · V_LL)      [kA]

PCC voltage (simplified Thevenin, lossless feeder assumption)
    ΔV_pu ≈ Q_injected / S_sc         (reactive-power dominated, X/R >> 1)

Non-detection zone (NDZ) for passive islanding detection
    The NDZ is the range of active/reactive power mismatch (ΔP, ΔQ) at the
    point of islanding for which passive methods (OV/UV, OF/UF) fail to
    detect the island.  Per IEC 62116 the NDZ is a rectangle in ΔP/ΔQ space:

        |ΔP/P_load| < (f_max/f0 - 1) / Qf    (frequency dimension)
        |ΔQ/Q_load| bounded by V trip limits   (voltage dimension)

    where Qf is the quality factor of the local load (typically 2.5).
"""

import math
from typing import Dict


class GridInterface:
    """
    Point-of-common-coupling (PCC) analysis for a grid-tied PCS.

    Parameters
    ----------
    grid_voltage_kv : float
        Nominal grid line-to-line voltage in kV at the PCC.
    grid_freq_hz : float
        Nominal grid frequency in Hz (50 or 60).
    short_circuit_mva : float
        Three-phase short-circuit apparent power at the PCC in MVA.
    xr_ratio : float, optional
        X/R ratio of the source impedance (default 10, typical MV network).
    """

    def __init__(
        self,
        grid_voltage_kv: float,
        grid_freq_hz: float,
        short_circuit_mva: float,
        xr_ratio: float = 10.0,
    ) -> None:
        if grid_voltage_kv <= 0:
            raise ValueError("grid_voltage_kv must be positive")
        if grid_freq_hz not in (50.0, 60.0) and not (45 < grid_freq_hz < 65):
            raise ValueError("grid_freq_hz should be near 50 or 60 Hz")
        if short_circuit_mva <= 0:
            raise ValueError("short_circuit_mva must be positive")

        self.grid_voltage_kv    = grid_voltage_kv
        self.grid_freq_hz       = grid_freq_hz
        self.short_circuit_mva  = short_circuit_mva
        self.xr_ratio           = xr_ratio

    # ------------------------------------------------------------------
    # Derived impedance properties
    # ------------------------------------------------------------------

    @property
    def z_source_ohm(self) -> complex:
        """
        Source impedance at the PCC (Thevenin equivalent) in ohms.

        Z_s = V^2 / S_sc    (complex, using X/R ratio)
        """
        z_mag = (self.grid_voltage_kv * 1e3) ** 2 / (self.short_circuit_mva * 1e6)
        angle = math.atan(self.xr_ratio)          # arctan(X/R)
        r_s = z_mag * math.cos(angle)
        x_s = z_mag * math.sin(angle)
        return complex(r_s, x_s)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def fault_level_kA(self) -> float:
        """
        Symmetrical (three-phase bolted) fault current at the PCC.

        I_fault = S_sc / (√3 · V_LL_kV)    [kA]

        Returns
        -------
        float
            Fault current in kA (RMS).
        """
        return self.short_circuit_mva / (math.sqrt(3) * self.grid_voltage_kv)

    def pcc_voltage_pu(self, injected_kvar: float, injected_kw: float = 0.0) -> float:
        """
        Steady-state PCC voltage in per-unit due to PCS power injection.

        Uses the full Thevenin voltage-rise formula (ΔV ≈ RP + XQ) / V²:
            ΔV_pu = (R·P + X·Q) / V_base²

        Positive Q injection raises voltage (capacitive convention).

        Parameters
        ----------
        injected_kvar : float
            Reactive power injected at PCC in kvar (+ = capacitive / leading).
        injected_kw : float, optional
            Active power injected at PCC in kW (+ = exporting to grid).

        Returns
        -------
        float
            PCC voltage in per-unit (pre-injection voltage = 1.0 pu assumed).
        """
        p_mw = injected_kw   / 1_000
        q_mvar = injected_kvar / 1_000
        v_kv2 = self.grid_voltage_kv ** 2
        z = self.z_source_ohm
        # Convert ohms to per-unit on the natural base (V²/S_sc)
        # ΔV_pu = (R·P + X·Q) / V² using SI units directly
        delta_v_kv = (z.real * p_mw + z.imag * q_mvar) / self.grid_voltage_kv
        delta_v_pu = delta_v_kv / self.grid_voltage_kv
        return 1.0 + delta_v_pu

    def islanding_detection_ndz(
        self,
        load_quality_factor: float = 2.5,
        v_trip_low_pu: float = 0.88,
        v_trip_high_pu: float = 1.10,
        f_trip_low_hz: float | None = None,
        f_trip_high_hz: float | None = None,
    ) -> Dict[str, float]:
        """
        Non-Detection Zone (NDZ) for passive islanding detection.

        The NDZ is the region in (ΔP, ΔQ) space — expressed as a fraction of
        rated load — where OV/UV and OF/UF relays fail to detect islanding.

        Method follows IEC 62116 Annex B.

        Parameters
        ----------
        load_quality_factor : float
            Quality factor Qf of the local RLC resonant load (default 2.5).
        v_trip_low_pu, v_trip_high_pu : float
            Under/over-voltage trip thresholds in p.u.
        f_trip_low_hz, f_trip_high_hz : float or None
            Frequency trip thresholds.  Defaults to ±3.5 % of nominal.

        Returns
        -------
        dict with keys:
            delta_p_low_pu   — lower bound of ΔP/P_load (< 0, power deficit)
            delta_p_high_pu  — upper bound of ΔP/P_load (> 0, power surplus)
            delta_q_low_pu   — lower bound of ΔQ/Q_load
            delta_q_high_pu  — upper bound of ΔQ/Q_load
            ndz_area_pu2     — area of NDZ rectangle in normalised units²
            f0_hz, v_trip_low_pu, v_trip_high_pu, f_trip_low_hz, f_trip_high_hz
        """
        f0 = self.grid_freq_hz

        if f_trip_low_hz is None:
            f_trip_low_hz  = f0 * (1 - 0.035)
        if f_trip_high_hz is None:
            f_trip_high_hz = f0 * (1 + 0.035)

        # --- Frequency dimension of NDZ ---
        # At resonance the island frequency drifts until load Q changes enough
        # to trip OF/UF.  The normalised power mismatch limits are:
        #   ΔP/P = ± (Δf/f0) / Qf   per Ropp et al. (2000)
        delta_f_low  = (f_trip_low_hz  - f0) / f0     # negative
        delta_f_high = (f_trip_high_hz - f0) / f0     # positive
        dp_low  = delta_f_low  / load_quality_factor
        dp_high = delta_f_high / load_quality_factor

        # --- Voltage dimension of NDZ ---
        # Reactive power mismatch that keeps V within trip limits.
        # ΔQ/Q_load ≈ (V_trip² - 1) / (2 · Qf)  (linearised)
        dq_low  = (v_trip_low_pu  ** 2 - 1.0) / (2 * load_quality_factor)
        dq_high = (v_trip_high_pu ** 2 - 1.0) / (2 * load_quality_factor)

        ndz_area = (dp_high - dp_low) * (dq_high - dq_low)

        return {
            "delta_p_low_pu":   round(dp_low,  6),
            "delta_p_high_pu":  round(dp_high, 6),
            "delta_q_low_pu":   round(dq_low,  6),
            "delta_q_high_pu":  round(dq_high, 6),
            "ndz_area_pu2":     round(ndz_area, 8),
            "f0_hz":            f0,
            "v_trip_low_pu":    v_trip_low_pu,
            "v_trip_high_pu":   v_trip_high_pu,
            "f_trip_low_hz":    f_trip_low_hz,
            "f_trip_high_hz":   f_trip_high_hz,
            "load_quality_factor": load_quality_factor,
        }

    # ------------------------------------------------------------------
    # Dunder helpers
    # ------------------------------------------------------------------

    def __repr__(self) -> str:
        return (
            f"GridInterface(grid_voltage_kv={self.grid_voltage_kv}, "
            f"grid_freq_hz={self.grid_freq_hz}, "
            f"short_circuit_mva={self.short_circuit_mva})"
        )
