"""
pcs.control.grid_following
==========================
Grid-following control for a VSC connected to a stiff AC grid.

Two classes are provided:

PLLController
    Synchronous Reference Frame PLL (SRF-PLL).  Tracks grid voltage angle θ
    by aligning the d-axis of the rotating frame with the grid voltage vector
    and driving vq → 0 via a PI compensator.

    State variables (persistent across calls): θ (angle), integrator state ε_i.

CurrentController
    PI-based inner current controller in the dq rotating frame.
    Implements cross-coupling decoupling and voltage feed-forward for fast,
    non-interacting d-axis (active) and q-axis (reactive) current tracking.

    State variables: integral errors for both d and q axes.

References
----------
- Teodorescu, Liserre, Rodriguez, "Grid Converters for Photovoltaic and
  Wind Power Systems," Wiley-IEEE, 2011.
- Blaabjerg et al., "Overview of Control and Grid Synchronization for
  Distributed Power Generation Systems," IEEE Trans. Ind. Electron., 2006.
- Zmood & Holmes, "Stationary Frame Current Regulation of PWM Inverters with
  Zero Steady-State Error," IEEE Trans. Power Electron., 2003.
"""

from __future__ import annotations
import math


# ---------------------------------------------------------------------------
# Synchronous Reference Frame PLL
# ---------------------------------------------------------------------------

class PLLController:
    """
    SRF-PLL for grid synchronisation.

    The PLL operates on αβ-frame voltage components (Clarke-transformed) and
    rotates them into the dq frame using the current angle estimate.  The
    q-axis component vq is driven to zero by a PI loop, yielding the grid
    angle θ and an estimate of the grid angular frequency ω.

    Parameters
    ----------
    kp : float
        Proportional gain of the PLL PI compensator [rad/s per V].
    ki : float
        Integral gain [rad/s² per V].
    f_nominal_hz : float
        Nominal grid frequency for feed-forward [Hz].
    """

    def __init__(
        self,
        kp: float = 50.0,
        ki: float = 1_000.0,
        f_nominal_hz: float = 60.0,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.omega_ff = 2 * math.pi * f_nominal_hz  # feed-forward angular freq.

        # Internal state
        self._theta: float = 0.0        # current angle estimate [rad]
        self._epsilon_i: float = 0.0    # PI integrator state [rad/s]

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def phase_angle(self, v_alpha: float, v_beta: float, dt: float) -> float:
        """
        Update PLL and return current grid phase angle estimate.

        The SRF-PLL error signal is the q-axis projection of the voltage
        vector in the estimated rotating frame:
            vq = -v_alpha * sin(θ) + v_beta * cos(θ)

        For small errors: vq ≈ |V| · sin(θ_grid − θ_pll) ≈ |V| · Δθ

        Parameters
        ----------
        v_alpha, v_beta : float
            Stationary-frame (αβ) voltage components in volts.
        dt : float
            Sampling interval in seconds.

        Returns
        -------
        float
            Updated angle estimate θ in radians (wraps to [0, 2π)).
        """
        if dt <= 0:
            raise ValueError("dt must be positive")

        # Park transform using current angle estimate (q-axis error)
        vq = -v_alpha * math.sin(self._theta) + v_beta * math.cos(self._theta)

        # PI compensator
        self._epsilon_i += vq * dt
        omega = self.omega_ff + self.kp * vq + self.ki * self._epsilon_i

        # Integrate angle
        self._theta += omega * dt
        self._theta %= (2 * math.pi)

        return self._theta

    def frequency_estimate(
        self,
        theta_prev: float,
        theta_curr: float,
        dt: float,
    ) -> float:
        """
        Estimate instantaneous grid frequency from consecutive angle samples.

        Handles the 2π wrap-around that occurs when θ crosses 0/2π.

        Parameters
        ----------
        theta_prev : float
            Previous angle estimate in radians.
        theta_curr : float
            Current angle estimate in radians.
        dt : float
            Time step in seconds.

        Returns
        -------
        float
            Frequency estimate in Hz.
        """
        if dt <= 0:
            raise ValueError("dt must be positive")

        delta_theta = theta_curr - theta_prev
        # Unwrap: keep Δθ in (−π, π]
        if delta_theta > math.pi:
            delta_theta -= 2 * math.pi
        elif delta_theta < -math.pi:
            delta_theta += 2 * math.pi

        omega_est = delta_theta / dt
        return omega_est / (2 * math.pi)

    def reset(self, theta_init: float = 0.0) -> None:
        """Reset PLL state (e.g. after a fault)."""
        self._theta = theta_init % (2 * math.pi)
        self._epsilon_i = 0.0

    @property
    def theta(self) -> float:
        """Current angle estimate in radians."""
        return self._theta

    def __repr__(self) -> str:
        return f"PLLController(kp={self.kp}, ki={self.ki}, theta={self._theta:.4f} rad)"


# ---------------------------------------------------------------------------
# dq Current Controller
# ---------------------------------------------------------------------------

class CurrentController:
    """
    PI current controller in the synchronous (dq) reference frame with
    cross-coupling and voltage feed-forward decoupling.

    The d-axis controls active current (∝ active power) and the q-axis
    controls reactive current.  Decoupling terms cancel the coupling between
    axes introduced by the rotating frame:

        vd_ref = kp*(id_ref−id) + ki*∫(id_ref−id)dt + vd_ff − ω·L·iq
        vq_ref = kp*(iq_ref−iq) + ki*∫(iq_ref−iq)dt + vq_ff + ω·L·id

    where L is the filter inductance and ω is the grid angular frequency.

    Parameters
    ----------
    kp : float
        Proportional gain [V/A].
    ki : float
        Integral gain [V/(A·s)].
    v_dc : float
        DC-link voltage [V] — used as output voltage clamp ceiling.
    l_filter_mH : float
        AC-side filter inductance in mH (for cross-coupling decoupling).
    omega_grid : float
        Grid angular frequency in rad/s (default 2π·60).
    i_max : float
        Peak current limit in A for anti-windup.
    """

    def __init__(
        self,
        kp: float = 10.0,
        ki: float = 500.0,
        v_dc: float = 800.0,
        l_filter_mH: float = 1.0,
        omega_grid: float | None = None,
        i_max: float = 2_000.0,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.v_dc = v_dc
        self.l_filter_H = l_filter_mH * 1e-3
        self.omega = omega_grid if omega_grid is not None else 2 * math.pi * 60
        self.i_max = i_max

        # Integral error states
        self._int_d: float = 0.0
        self._int_q: float = 0.0

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def dq_current_control(
        self,
        id_ref: float,
        iq_ref: float,
        id_meas: float,
        iq_meas: float,
        dt: float,
        vd_ff: float = 0.0,
        vq_ff: float = 0.0,
    ) -> tuple[float, float]:
        """
        One-step PI current controller update in the dq frame.

        Anti-windup is implemented via conditional integration: the integrator
        is frozen on the axis whose output is saturated.

        Parameters
        ----------
        id_ref, iq_ref : float
            d- and q-axis current references in amperes.
        id_meas, iq_meas : float
            Measured d- and q-axis currents in amperes.
        dt : float
            Sampling interval in seconds.
        vd_ff, vq_ff : float, optional
            Voltage feed-forward components (typically the measured grid
            d/q voltages) in volts.

        Returns
        -------
        (vd_ref, vq_ref) : tuple of float
            Voltage references in the dq frame in volts.
        """
        if dt <= 0:
            raise ValueError("dt must be positive")

        # Clamp references to current limit
        id_ref = max(-self.i_max, min(self.i_max, id_ref))
        iq_ref = max(-self.i_max, min(self.i_max, iq_ref))

        err_d = id_ref - id_meas
        err_q = iq_ref - iq_meas

        # Cross-coupling decoupling terms
        decouple_d = -self.omega * self.l_filter_H * iq_meas
        decouple_q =  self.omega * self.l_filter_H * id_meas

        # Unclamped PI outputs
        v_sat = self.v_dc / 2.0   # peak phase voltage ceiling

        vd_raw = self.kp * err_d + self.ki * self._int_d + vd_ff + decouple_d
        vq_raw = self.kp * err_q + self.ki * self._int_q + vq_ff + decouple_q

        # Clamp outputs (space-vector ceiling: |v_dq| ≤ V_dc/√3)
        v_mag = math.sqrt(vd_raw**2 + vq_raw**2)
        v_limit = self.v_dc / math.sqrt(3)
        if v_mag > v_limit and v_mag > 0:
            scale = v_limit / v_mag
            vd_ref = vd_raw * scale
            vq_ref = vq_raw * scale
        else:
            vd_ref = vd_raw
            vq_ref = vq_raw

        # Anti-windup: only integrate if output is not clamped on that axis
        if abs(vd_raw) < v_sat:
            self._int_d += err_d * dt
        if abs(vq_raw) < v_sat:
            self._int_q += err_q * dt

        return float(vd_ref), float(vq_ref)

    def reset(self) -> None:
        """Reset integrator states."""
        self._int_d = 0.0
        self._int_q = 0.0

    def __repr__(self) -> str:
        return (
            f"CurrentController(kp={self.kp}, ki={self.ki}, "
            f"v_dc={self.v_dc}, l_filter_mH={self.l_filter_H*1e3})"
        )
