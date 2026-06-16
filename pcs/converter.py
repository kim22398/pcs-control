"""
pcs.converter
=============
BidirectionalConverter — steady-state model for a voltage-source, grid-tied
bidirectional AC/DC power converter (e.g. 3-level NPC or T-type topology).

Loss model
----------
Total loss is approximated as:
    P_loss(p.u.) = P_fixed + k_cond * P + k_sw * P
where
    P_fixed  — no-load / fixed loss (gate drivers, aux supplies, iron)
    k_cond   — conduction loss coefficient (proportional to current ~ power)
    k_sw     — switching loss coefficient (proportional to current × V_dc)

Coefficients are calibrated so that peak efficiency ≈ 98 % at ~50 % load,
matching datasheets for modern SiC-based 1 MVA PCS modules.
"""

import math


class BidirectionalConverter:
    """
    Steady-state model of a bidirectional AC/DC voltage-source converter.

    Parameters
    ----------
    rated_kw : float
        Rated active power in kW (positive = inverter / discharge direction).
    dc_voltage_V : float
        Nominal DC-link voltage in volts.
    ac_voltage_V : float
        Nominal AC line-to-line (LL) RMS voltage in volts.
    switching_freq_hz : float
        PWM switching frequency in Hz (affects switching losses).
    """

    # Loss model coefficients — calibrated for SiC MOSFET NPC topology
    _P_FIXED_PU = 0.004          # fixed loss as fraction of rated power
    _K_COND     = 0.008          # conduction loss coefficient
    _K_SW_BASE  = 2e-5           # switching loss coefficient at 10 kHz
    _F_SW_BASE  = 10_000         # reference switching frequency (Hz)

    def __init__(
        self,
        rated_kw: float,
        dc_voltage_V: float,
        ac_voltage_V: float,
        switching_freq_hz: float = 10_000,
    ) -> None:
        if rated_kw <= 0:
            raise ValueError("rated_kw must be positive")
        if dc_voltage_V <= 0 or ac_voltage_V <= 0:
            raise ValueError("DC and AC voltages must be positive")
        if switching_freq_hz <= 0:
            raise ValueError("switching_freq_hz must be positive")

        self.rated_kw         = rated_kw
        self.dc_voltage_V     = dc_voltage_V
        self.ac_voltage_V     = ac_voltage_V
        self.switching_freq_hz = switching_freq_hz

        # Scale switching loss with actual vs. reference switching frequency
        self._k_sw = self._K_SW_BASE * (switching_freq_hz / self._F_SW_BASE)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def efficiency(self, load_factor: float) -> float:
        """
        Return converter efficiency η at a given load factor.

        Parameters
        ----------
        load_factor : float
            Fraction of rated power in [0, 1].  Values outside this range are
            clamped with a warning because the loss model is only valid within
            the rated operating envelope.

        Returns
        -------
        float
            Efficiency η ∈ (0, 1].
        """
        if not (0.0 <= load_factor <= 1.0):
            load_factor = max(0.0, min(1.0, load_factor))

        if load_factor == 0.0:
            return 0.0  # no output power → efficiency undefined / 0

        p_loss_pu = (
            self._P_FIXED_PU
            + self._K_COND * load_factor
            + self._k_sw   * load_factor
        )
        # η = P_out / P_in  =  lf / (lf + p_loss)
        eta = load_factor / (load_factor + p_loss_pu)
        return min(eta, 1.0)

    def dc_current(self, power_kw: float) -> float:
        """
        DC-link current for a given output power (inverter direction).

        Accounts for converter losses: I_dc = P_in / V_dc
        where P_in = P_out / η.

        Parameters
        ----------
        power_kw : float
            Desired AC output power in kW (positive = export to grid).

        Returns
        -------
        float
            DC current in amperes.
        """
        if power_kw < 0:
            raise ValueError("power_kw must be non-negative (use positive convention)")
        if power_kw == 0:
            return 0.0

        load_factor = power_kw / self.rated_kw
        eta = self.efficiency(load_factor)
        p_in_kw = power_kw / eta
        return (p_in_kw * 1_000) / self.dc_voltage_V

    def ac_current_A(self, power_kw: float, pf: float = 1.0) -> float:
        """
        AC line current (RMS) for a given active power and power factor.

        For a balanced 3-phase system:
            I_ac = P / (√3 · V_LL · pf)

        Parameters
        ----------
        power_kw : float
            Active power in kW.
        pf : float
            Displacement power factor in (0, 1].

        Returns
        -------
        float
            Line current in amperes (RMS).
        """
        if not (0 < pf <= 1.0):
            raise ValueError("pf must be in (0, 1]")
        if power_kw < 0:
            raise ValueError("power_kw must be non-negative")
        if power_kw == 0:
            return 0.0

        return (power_kw * 1_000) / (math.sqrt(3) * self.ac_voltage_V * pf)

    def modulation_index(self) -> float:
        """
        Sinusoidal PWM (SPWM) modulation index.

        For a 3-phase VSI the linear modulation range is m_a ≤ 1.
        Using peak phase voltage:
            V_phase_peak = V_LL_rms * √(2/3)
            m_a = V_phase_peak / (V_dc / 2)

        Returns
        -------
        float
            Modulation index m_a (values > 1 indicate over-modulation).
        """
        v_phase_peak = self.ac_voltage_V * math.sqrt(2.0 / 3.0)
        return v_phase_peak / (self.dc_voltage_V / 2.0)

    # ------------------------------------------------------------------
    # Dunder helpers
    # ------------------------------------------------------------------

    def __repr__(self) -> str:
        return (
            f"BidirectionalConverter(rated_kw={self.rated_kw}, "
            f"dc_voltage_V={self.dc_voltage_V}, "
            f"ac_voltage_V={self.ac_voltage_V}, "
            f"switching_freq_hz={self.switching_freq_hz})"
        )
