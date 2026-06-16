"""
pcs.control.droop
=================
DroopController — P-f and Q-V droop for grid-forming PCS.

Background
----------
Grid-forming inverters use droop characteristics to share load autonomously
without a communication link, mimicking synchronous generator governor and
AVR behaviour.

P-f droop (governor analogy)
    f = f0 - Rp * (P - P0)
    Rp = (droop_pct / 100) * f0 / P_rated   [Hz / kW]

Q-V droop (AVR analogy)
    V = V0 - Rq * (Q - Q0)
    Rq = (droop_pct / 100) * V_rated / Q_rated   [pu / kvar]

Both characteristics are implemented as stateless mappings so they can be
embedded inside a time-domain simulation loop without hidden integrators.
Rate limiting and set-point ramping are left to the caller.

References
----------
- Guerrero et al., "Hierarchical Control of Droop-Controlled AC and DC
  Microgrids," IEEE Trans. Ind. Electron., 2011.
- IEEE 1547.4-2011, "Guide for Design, Operation, and Integration of
  Distributed Resource Island Systems."
- CIGRE WG C6.22, "Microgrids 1: Engineering, Economics & Experience," 2015.
"""

from __future__ import annotations


class DroopController:
    """
    Stateless P-f and Q-V droop controller for a grid-forming PCS.

    Parameters
    ----------
    rated_kw : float
        Rated active power in kW.
    rated_kvar : float
        Rated reactive power in kvar.
    rated_freq_hz : float
        Nominal grid frequency in Hz (50 or 60).
    rated_voltage_pu : float
        Nominal voltage magnitude in per-unit (typically 1.0).
    f_deadband_hz : float, optional
        Frequency deadband ± around f0; droop is inactive within this band.
        Default 0 (no deadband).
    v_deadband_pu : float, optional
        Voltage deadband ± around V0; Q droop is inactive within this band.
        Default 0.
    """

    def __init__(
        self,
        rated_kw: float,
        rated_kvar: float,
        rated_freq_hz: float = 60.0,
        rated_voltage_pu: float = 1.0,
        f_deadband_hz: float = 0.0,
        v_deadband_pu: float = 0.0,
    ) -> None:
        if rated_kw <= 0:
            raise ValueError("rated_kw must be positive")
        if rated_kvar <= 0:
            raise ValueError("rated_kvar must be positive")
        if not (45 < rated_freq_hz < 65):
            raise ValueError("rated_freq_hz must be 50 or 60 Hz")
        if rated_voltage_pu <= 0:
            raise ValueError("rated_voltage_pu must be positive")

        self.rated_kw        = rated_kw
        self.rated_kvar      = rated_kvar
        self.rated_freq_hz   = rated_freq_hz
        self.rated_voltage_pu = rated_voltage_pu
        self.f_deadband_hz   = f_deadband_hz
        self.v_deadband_pu   = v_deadband_pu

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _p_droop_slope(self, droop_pct: float) -> float:
        """P-f droop slope Rp [kW / Hz]."""
        # droop_pct: e.g. 5 means 5 % frequency change = 100 % power change
        # Rp = P_rated / (droop_pct/100 * f0)  → [kW/Hz]
        return self.rated_kw / (droop_pct / 100.0 * self.rated_freq_hz)

    def _q_droop_slope(self, droop_pct: float) -> float:
        """Q-V droop slope Rq [kvar / pu]."""
        return self.rated_kvar / (droop_pct / 100.0 * self.rated_voltage_pu)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def frequency_droop(
        self,
        f_measured: float,
        p_setpoint: float,
        droop_pct: float = 5.0,
    ) -> float:
        """
        Compute active power output from P-f droop characteristic.

        The controller raises active power when frequency falls below f0 and
        lowers it when frequency rises (governor / load-sharing behaviour).

            P_out = P_setpoint + Rp * (f0 - f_measured)

        The result is clamped to [0, rated_kw].

        Parameters
        ----------
        f_measured : float
            Measured (or estimated) grid / island frequency in Hz.
        p_setpoint : float
            Active power set-point in kW (dispatch target at f0).
        droop_pct : float
            Droop coefficient in percent.  5 % is typical for BESS PCS.

        Returns
        -------
        float
            Active power output command in kW.
        """
        if droop_pct <= 0:
            raise ValueError("droop_pct must be positive")

        delta_f = f_measured - self.rated_freq_hz

        # Apply deadband
        if abs(delta_f) <= self.f_deadband_hz:
            delta_f = 0.0

        rp = self._p_droop_slope(droop_pct)
        p_out = p_setpoint - rp * delta_f   # lower f → higher P output
        return float(max(0.0, min(self.rated_kw, p_out)))

    def voltage_droop(
        self,
        v_measured: float,
        q_setpoint: float,
        droop_pct: float = 5.0,
    ) -> float:
        """
        Compute reactive power output from Q-V droop characteristic.

        The controller injects positive Q (capacitive) when voltage sags and
        absorbs Q when voltage is high.

            Q_out = Q_setpoint + Rq * (V0 - V_measured)

        The result is clamped to [-rated_kvar, +rated_kvar].

        Parameters
        ----------
        v_measured : float
            Measured PCC voltage in per-unit.
        q_setpoint : float
            Reactive power set-point in kvar at rated voltage.
        droop_pct : float
            Q-V droop coefficient in percent.  5 % means rated Q is reached
            at a 5 % voltage deviation from the set-point.

        Returns
        -------
        float
            Reactive power output command in kvar (+ = capacitive).
        """
        if droop_pct <= 0:
            raise ValueError("droop_pct must be positive")

        delta_v = v_measured - self.rated_voltage_pu

        # Apply deadband
        if abs(delta_v) <= self.v_deadband_pu:
            delta_v = 0.0

        rq = self._q_droop_slope(droop_pct)
        q_out = q_setpoint - rq * delta_v   # lower V → higher Q injection
        return float(max(-self.rated_kvar, min(self.rated_kvar, q_out)))

    def operating_point(
        self,
        f_measured: float,
        v_measured: float,
        p_setpoint: float,
        q_setpoint: float,
        p_droop_pct: float = 5.0,
        q_droop_pct: float = 5.0,
    ) -> dict:
        """
        Return the full droop operating point as a dict.

        Convenience wrapper that calls both droop characteristics and returns
        a labelled dict suitable for logging / pandas DataFrames.
        """
        p_out = self.frequency_droop(f_measured, p_setpoint, p_droop_pct)
        q_out = self.voltage_droop(v_measured, q_setpoint, q_droop_pct)
        return {
            "f_hz":       f_measured,
            "v_pu":       v_measured,
            "p_kw":       p_out,
            "q_kvar":     q_out,
            "p_load_pct": 100.0 * p_out / self.rated_kw,
            "q_load_pct": 100.0 * q_out / self.rated_kvar,
        }

    def __repr__(self) -> str:
        return (
            f"DroopController(rated_kw={self.rated_kw}, "
            f"rated_kvar={self.rated_kvar}, "
            f"rated_freq_hz={self.rated_freq_hz}, "
            f"rated_voltage_pu={self.rated_voltage_pu})"
        )
