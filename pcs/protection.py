"""
pcs.protection
==============
PCSProtection — trip logic for grid-tied PCS protection functions.

Covers IEEE 1547-2018 Categories A & B abnormal operating performance
requirements, IEC 62116:2014 anti-islanding, and basic DC protection.

All methods return a (trip: bool, reason: str) tuple so callers can
evaluate all functions independently and log the reason string.

Trip thresholds
---------------
IEEE 1547-2018 Table 3 (60 Hz system, Category A defaults):
    Voltage:   0.88–1.10 pu continuous operating range
    Frequency: 59.3–60.5 Hz
    ROCOF:     typically 0.5–2.0 Hz/s per utility specification

References
----------
- IEEE Std 1547-2018, "Standard for Interconnection and Interoperability
  of Distributed Energy Resources with Associated Electric Power Systems
  Interfaces."
- IEC 62116:2014, "Test Procedure of Islanding Prevention Measures."
- NERC PRC-024-3, "Generator Frequency and Voltage Protective Relay
  Settings."
"""

from __future__ import annotations
from typing import List, Tuple


# Type alias
TripResult = Tuple[bool, str]


class PCSProtection:
    """
    IEEE 1547-2018 / IEC 62116 protection functions for grid-tied PCS.

    This class is intentionally stateless for the instantaneous functions
    (voltage, frequency, DC overvoltage, current limit).  The ROCOF function
    holds no internal state — the caller supplies consecutive frequency
    samples.

    All trip thresholds can be overridden at construction time to accommodate
    different utility interconnection agreements.
    """

    def __init__(self) -> None:
        pass   # stateless; thresholds passed per-call or overridden in subclass

    # ------------------------------------------------------------------
    # Voltage protection
    # ------------------------------------------------------------------

    def over_under_voltage(
        self,
        v_pu: float,
        limits: Tuple[float, float] = (0.88, 1.10),
    ) -> TripResult:
        """
        Evaluate over- and under-voltage (OV/UV) trip condition.

        IEEE 1547-2018 §7.4 specifies trip windows; the default limits match
        the widest Category A continuous operating range.

        Parameters
        ----------
        v_pu : float
            Measured voltage at PCC in per-unit.
        limits : (v_low, v_high), optional
            Voltage trip thresholds in per-unit.

        Returns
        -------
        (trip, reason)
            trip=True and reason='UV' if v_pu < v_low,
            trip=True and reason='OV' if v_pu > v_high,
            trip=False and reason='OK' otherwise.
        """
        v_low, v_high = limits
        if v_pu < v_low:
            return True, f"UV: {v_pu:.4f} pu < {v_low} pu"
        if v_pu > v_high:
            return True, f"OV: {v_pu:.4f} pu > {v_high} pu"
        return False, "OK"

    # ------------------------------------------------------------------
    # Frequency protection
    # ------------------------------------------------------------------

    def over_under_frequency(
        self,
        f_hz: float,
        limits: Tuple[float, float] = (59.3, 60.5),
    ) -> TripResult:
        """
        Evaluate over- and under-frequency (OF/UF) trip condition.

        Default limits match IEEE 1547-2018 Table 4 Category A for 60 Hz.

        Parameters
        ----------
        f_hz : float
            Measured frequency in Hz.
        limits : (f_low, f_high), optional
            Frequency trip thresholds in Hz.

        Returns
        -------
        (trip, reason)
        """
        f_low, f_high = limits
        if f_hz < f_low:
            return True, f"UF: {f_hz:.4f} Hz < {f_low} Hz"
        if f_hz > f_high:
            return True, f"OF: {f_hz:.4f} Hz > {f_high} Hz"
        return False, "OK"

    # ------------------------------------------------------------------
    # ROCOF protection
    # ------------------------------------------------------------------

    def rate_of_change_of_frequency(
        self,
        f_samples: List[float],
        dt: float,
        threshold: float = 1.0,
    ) -> TripResult:
        """
        Evaluate Rate-of-Change-of-Frequency (ROCOF) trip.

        ROCOF is estimated from consecutive frequency samples using a
        first-difference derivative:
            df/dt[k] = (f[k] − f[k-1]) / dt

        The peak absolute ROCOF across all sample pairs is compared against
        the threshold.

        Parameters
        ----------
        f_samples : list of float
            Sequence of at least 2 frequency measurements in Hz,
            uniformly spaced at interval dt.
        dt : float
            Sample interval in seconds (e.g. 0.02 for one cycle at 50 Hz).
        threshold : float
            ROCOF trip threshold in Hz/s (default 1.0 Hz/s;
            IEEE 1547-2018 allows 0.5–3.0 Hz/s per interconnection agreement).

        Returns
        -------
        (trip, reason)
        """
        if len(f_samples) < 2:
            raise ValueError("f_samples must contain at least 2 elements")
        if dt <= 0:
            raise ValueError("dt must be positive")

        rocof_values = [
            (f_samples[k] - f_samples[k - 1]) / dt
            for k in range(1, len(f_samples))
        ]
        max_rocof = max(abs(r) for r in rocof_values)
        peak_rocof = rocof_values[
            max((range(len(rocof_values))), key=lambda i: abs(rocof_values[i]))
        ]

        if max_rocof > threshold:
            return True, (
                f"ROCOF: {peak_rocof:+.4f} Hz/s exceeds ±{threshold} Hz/s"
            )
        return False, f"ROCOF OK: max |df/dt| = {max_rocof:.4f} Hz/s"

    # ------------------------------------------------------------------
    # DC overvoltage protection
    # ------------------------------------------------------------------

    def dc_overvoltage(self, v_dc: float, v_max: float) -> TripResult:
        """
        DC-link overvoltage protection.

        Relevant during regenerative braking, sudden load rejection, or
        failure of the DC-side converter.

        Parameters
        ----------
        v_dc : float
            Measured DC-link voltage in volts.
        v_max : float
            DC overvoltage trip level in volts.

        Returns
        -------
        (trip, reason)
        """
        if v_dc > v_max:
            return True, f"DC OV: {v_dc:.1f} V > {v_max:.1f} V"
        return False, f"DC OK: {v_dc:.1f} V"

    # ------------------------------------------------------------------
    # AC current limit / overcurrent protection
    # ------------------------------------------------------------------

    def current_limit(self, i_measured: float, i_max: float) -> TripResult:
        """
        AC overcurrent / hardware current limit check.

        Typically set at 110–120 % of rated current for time-delayed
        overcurrent (50 / 51) or at 200 % for instantaneous (50I) protection.

        Parameters
        ----------
        i_measured : float
            Measured peak or RMS AC current in amperes.
        i_max : float
            Maximum allowable current in amperes.

        Returns
        -------
        (trip, reason)
        """
        if i_measured > i_max:
            return True, f"OC: {i_measured:.1f} A > {i_max:.1f} A limit"
        return False, f"Current OK: {i_measured:.1f} A"

    # ------------------------------------------------------------------
    # Convenience: evaluate all protections at once
    # ------------------------------------------------------------------

    def evaluate_all(
        self,
        v_pu: float,
        f_hz: float,
        v_dc: float,
        v_dc_max: float,
        i_meas: float,
        i_max: float,
        f_samples: List[float] | None = None,
        dt: float = 0.02,
        rocof_threshold: float = 1.0,
        v_limits: Tuple[float, float] = (0.88, 1.10),
        f_limits: Tuple[float, float] = (59.3, 60.5),
    ) -> dict:
        """
        Evaluate all protection functions and return a summary dict.

        Returns
        -------
        dict with keys matching each protection function name.
        """
        results: dict = {}

        trip_v, reason_v = self.over_under_voltage(v_pu, v_limits)
        results["voltage"]  = {"trip": trip_v,  "reason": reason_v}

        trip_f, reason_f = self.over_under_frequency(f_hz, f_limits)
        results["frequency"] = {"trip": trip_f, "reason": reason_f}

        trip_dc, reason_dc = self.dc_overvoltage(v_dc, v_dc_max)
        results["dc_voltage"] = {"trip": trip_dc, "reason": reason_dc}

        trip_oc, reason_oc = self.current_limit(i_meas, i_max)
        results["overcurrent"] = {"trip": trip_oc, "reason": reason_oc}

        if f_samples is not None and len(f_samples) >= 2:
            trip_r, reason_r = self.rate_of_change_of_frequency(
                f_samples, dt, rocof_threshold
            )
            results["rocof"] = {"trip": trip_r, "reason": reason_r}

        results["any_trip"] = any(v["trip"] for v in results.values() if isinstance(v, dict))
        return results
