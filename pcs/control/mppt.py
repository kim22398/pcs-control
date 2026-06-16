"""
pcs.control.mppt
================
MPPTController — Perturb & Observe (P&O) maximum power point tracking.

Algorithm
---------
The P&O algorithm perturbs the PV array reference voltage by a fixed step
each control cycle and observes the change in output power.  If power
increased, the perturbation direction is maintained; if it decreased, the
direction is reversed.

    if P[k] > P[k-1]:
        if V[k] > V[k-1]:  V_ref[k+1] = V[k] + ΔV   (continue right)
        else:               V_ref[k+1] = V[k] − ΔV   (continue left)
    else:
        if V[k] > V[k-1]:  V_ref[k+1] = V[k] − ΔV   (reverse)
        else:               V_ref[k+1] = V[k] + ΔV   (reverse)

This is a variable-reference P&O variant where the output is a voltage
set-point fed to the DC/DC converter (or the outer loop of the PCS control).

Limitations
-----------
- Standard P&O oscillates ±ΔV around the true MPP in steady state.
- Under rapidly changing irradiance the algorithm may track a local maximum.
- For production use, Incremental Conductance (InC) or variable-step P&O
  (VS-P&O) should be considered.

References
----------
- Esram & Chapman, "Comparison of Photovoltaic Array MPPT Techniques," IEEE
  Trans. Energy Convers., 2007.
- Femia et al., "Optimization of Perturb and Observe MPPT Method," IEEE
  Trans. Power Electron., 2005.
"""

from __future__ import annotations
from typing import List


class MPPTController:
    """
    Perturb & Observe MPPT controller.

    Parameters
    ----------
    step_size_V : float
        Perturbation voltage step ΔV in volts.
    v_min : float
        Minimum allowable voltage reference (lower bound).
    v_max : float
        Maximum allowable voltage reference (upper bound).
    v_init : float, optional
        Initial voltage reference.  Defaults to midpoint of [v_min, v_max].
    """

    def __init__(
        self,
        step_size_V: float = 1.0,
        v_min: float = 200.0,
        v_max: float = 800.0,
        v_init: float | None = None,
    ) -> None:
        if step_size_V <= 0:
            raise ValueError("step_size_V must be positive")
        if v_min >= v_max:
            raise ValueError("v_min must be less than v_max")

        self.step_size_V = step_size_V
        self.v_min = v_min
        self.v_max = v_max

        self._v_ref   = v_init if v_init is not None else (v_min + v_max) / 2.0
        self._v_prev  = self._v_ref
        self._p_prev  = 0.0
        self._initialised = False   # need one real sample before perturbing

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def update(self, v_pv: float, p_pv: float) -> float:
        """
        Process one PV sample and return the new voltage reference.

        Call this once per MPPT control period (typically 10–100 ms for PV).

        Parameters
        ----------
        v_pv : float
            Measured PV array terminal voltage in volts.
        p_pv : float
            Measured PV array output power in watts.

        Returns
        -------
        float
            New voltage reference V_ref in volts.
        """
        if not self._initialised:
            # First call: seed previous values, do not perturb yet
            self._v_prev = v_pv
            self._p_prev = p_pv
            self._initialised = True
            return self._v_ref

        delta_p = p_pv - self._p_prev
        delta_v = v_pv - self._v_prev

        # Core P&O decision
        if delta_p == 0:
            # No change — stay put
            pass
        elif delta_p > 0:
            # Power increased — continue in the same direction
            if delta_v >= 0:
                self._v_ref += self.step_size_V
            else:
                self._v_ref -= self.step_size_V
        else:
            # Power decreased — reverse direction
            if delta_v >= 0:
                self._v_ref -= self.step_size_V
            else:
                self._v_ref += self.step_size_V

        # Apply voltage limits
        self._v_ref = max(self.v_min, min(self.v_max, self._v_ref))

        # Update previous values
        self._v_prev = v_pv
        self._p_prev = p_pv

        return self._v_ref

    def track(
        self, v_pv_list: List[float], p_pv_list: List[float]
    ) -> List[dict]:
        """
        Offline analysis: run P&O over historical V/P arrays.

        Resets the controller state before running so results are
        reproducible regardless of previous calls to `update()`.

        Parameters
        ----------
        v_pv_list : list of float
            Sequence of measured PV voltages.
        p_pv_list : list of float
            Corresponding measured PV powers (same length).

        Returns
        -------
        list of dict
            One record per sample:  {'step': k, 'v_pv', 'p_pv', 'v_ref'}
        """
        if len(v_pv_list) != len(p_pv_list):
            raise ValueError("v_pv_list and p_pv_list must have the same length")

        self.reset()
        history: List[dict] = []

        for k, (v, p) in enumerate(zip(v_pv_list, p_pv_list)):
            v_ref = self.update(v, p)
            history.append({"step": k, "v_pv": v, "p_pv": p, "v_ref": v_ref})

        return history

    def reset(self) -> None:
        """Reset controller state to initial conditions."""
        self._v_ref       = (self.v_min + self.v_max) / 2.0
        self._v_prev      = self._v_ref
        self._p_prev      = 0.0
        self._initialised = False

    @property
    def v_ref(self) -> float:
        """Current voltage reference in volts."""
        return self._v_ref

    def __repr__(self) -> str:
        return (
            f"MPPTController(step_size_V={self.step_size_V}, "
            f"v_min={self.v_min}, v_max={self.v_max}, "
            f"v_ref={self._v_ref:.2f})"
        )
