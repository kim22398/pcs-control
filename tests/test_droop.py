"""
tests/test_droop.py
===================
pytest suite for pcs.control.droop.DroopController.

Covers:
- Constructor validation
- P-f droop: nominal, frequency dip, frequency rise, clamping
- Q-V droop: nominal, voltage sag, voltage surge, clamping
- Deadband behaviour
- operating_point convenience method
- Physical consistency (correct sign conventions, monotonicity)
"""

import math
import pytest
from pcs.control.droop import DroopController


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def ctrl():
    """Standard 1 MW / 600 kvar, 60 Hz droop controller."""
    return DroopController(
        rated_kw=1_000.0,
        rated_kvar=600.0,
        rated_freq_hz=60.0,
        rated_voltage_pu=1.0,
    )


@pytest.fixture
def ctrl_50hz():
    """600 kW / 300 kvar, 50 Hz controller for multi-frequency tests."""
    return DroopController(
        rated_kw=600.0,
        rated_kvar=300.0,
        rated_freq_hz=50.0,
        rated_voltage_pu=1.0,
    )


@pytest.fixture
def ctrl_deadband():
    """Controller with non-zero deadbands."""
    return DroopController(
        rated_kw=1_000.0,
        rated_kvar=600.0,
        rated_freq_hz=60.0,
        rated_voltage_pu=1.0,
        f_deadband_hz=0.05,
        v_deadband_pu=0.01,
    )


# ---------------------------------------------------------------------------
# Constructor validation
# ---------------------------------------------------------------------------

class TestConstructor:
    def test_valid_construction(self, ctrl):
        assert ctrl.rated_kw == 1_000.0
        assert ctrl.rated_kvar == 600.0
        assert ctrl.rated_freq_hz == 60.0

    def test_invalid_rated_kw(self):
        with pytest.raises(ValueError, match="rated_kw"):
            DroopController(rated_kw=-100, rated_kvar=600, rated_freq_hz=60)

    def test_invalid_rated_kvar(self):
        with pytest.raises(ValueError, match="rated_kvar"):
            DroopController(rated_kw=1000, rated_kvar=0, rated_freq_hz=60)

    def test_invalid_frequency(self):
        with pytest.raises(ValueError):
            DroopController(rated_kw=1000, rated_kvar=600, rated_freq_hz=30)

    def test_invalid_voltage(self):
        with pytest.raises(ValueError):
            DroopController(
                rated_kw=1000, rated_kvar=600, rated_freq_hz=60, rated_voltage_pu=-1
            )


# ---------------------------------------------------------------------------
# P-f droop
# ---------------------------------------------------------------------------

class TestFrequencyDroop:
    def test_at_nominal_frequency_no_deviation(self, ctrl):
        """At f0, output should equal the set-point."""
        p = ctrl.frequency_droop(f_measured=60.0, p_setpoint=500.0, droop_pct=5.0)
        assert p == pytest.approx(500.0, rel=1e-6)

    def test_frequency_dip_increases_power(self, ctrl):
        """Frequency drop below f0 must cause P to increase (governor response)."""
        p_base = ctrl.frequency_droop(60.0, 500.0, droop_pct=5.0)
        p_dip  = ctrl.frequency_droop(59.5, 500.0, droop_pct=5.0)
        assert p_dip > p_base

    def test_frequency_rise_decreases_power(self, ctrl):
        """Frequency above f0 must cause P to decrease."""
        p_base = ctrl.frequency_droop(60.0,  500.0, droop_pct=5.0)
        p_rise = ctrl.frequency_droop(60.5, 500.0, droop_pct=5.0)
        assert p_rise < p_base

    def test_full_droop_range(self, ctrl):
        """5 % droop: 100 % rated frequency deviation → 100 % rated power change."""
        # Rp = P_rated / (0.05 * f0) = 1000 / 3 kW/Hz
        # ΔP = Rp * Δf = (1000/3) * (0.05*60) = 1000 kW  (matches rated)
        p_full_dip = ctrl.frequency_droop(
            f_measured=60.0 - 0.05 * 60.0,   # 57 Hz
            p_setpoint=0.0,
            droop_pct=5.0,
        )
        assert p_full_dip == pytest.approx(ctrl.rated_kw, rel=1e-6)

    def test_output_clamped_at_zero(self, ctrl):
        """Output must not go negative regardless of frequency."""
        p = ctrl.frequency_droop(f_measured=65.0, p_setpoint=0.0, droop_pct=5.0)
        assert p == pytest.approx(0.0, abs=1e-9)

    def test_output_clamped_at_rated(self, ctrl):
        """Output must not exceed rated_kw regardless of frequency."""
        p = ctrl.frequency_droop(f_measured=50.0, p_setpoint=1_000.0, droop_pct=5.0)
        assert p <= ctrl.rated_kw + 1e-9

    def test_droop_pct_zero_raises(self, ctrl):
        with pytest.raises(ValueError):
            ctrl.frequency_droop(60.0, 500.0, droop_pct=0.0)

    def test_monotonically_decreasing_with_frequency(self, ctrl):
        """Power must decrease monotonically as frequency increases."""
        freqs = [59.0, 59.5, 59.8, 60.0, 60.2, 60.5, 61.0]
        powers = [ctrl.frequency_droop(f, 500.0, droop_pct=5.0) for f in freqs]
        for i in range(len(powers) - 1):
            assert powers[i] >= powers[i + 1] - 1e-9, (
                f"Not monotone: P({freqs[i]})={powers[i]:.2f} < P({freqs[i+1]})={powers[i+1]:.2f}"
            )

    def test_50hz_system(self, ctrl_50hz):
        """Droop controller works correctly on a 50 Hz system."""
        p = ctrl_50hz.frequency_droop(50.0, 300.0, droop_pct=4.0)
        assert p == pytest.approx(300.0, rel=1e-6)

    def test_droop_slope_value(self, ctrl):
        """Verify droop slope Rp = P_rated / (d/100 * f0) against analytical value."""
        droop_pct = 4.0
        rp_expected = ctrl.rated_kw / (droop_pct / 100.0 * ctrl.rated_freq_hz)
        delta_f = -1.0   # Hz below nominal
        p_expected = min(ctrl.rated_kw, max(0.0, 500.0 + rp_expected * abs(delta_f)))
        p_actual = ctrl.frequency_droop(
            f_measured=60.0 + delta_f, p_setpoint=500.0, droop_pct=droop_pct
        )
        assert p_actual == pytest.approx(p_expected, rel=1e-6)


# ---------------------------------------------------------------------------
# Q-V droop
# ---------------------------------------------------------------------------

class TestVoltageDroop:
    def test_at_rated_voltage_no_deviation(self, ctrl):
        """At V0=1.0 pu, output equals set-point."""
        q = ctrl.voltage_droop(v_measured=1.0, q_setpoint=0.0, droop_pct=5.0)
        assert q == pytest.approx(0.0, abs=1e-9)

    def test_voltage_sag_injects_positive_q(self, ctrl):
        """Voltage below V0 → positive Q injection (capacitive support)."""
        q = ctrl.voltage_droop(v_measured=0.95, q_setpoint=0.0, droop_pct=5.0)
        assert q > 0.0, "Expected Q > 0 (capacitive) for low voltage"

    def test_voltage_surge_absorbs_q(self, ctrl):
        """Voltage above V0 → negative Q (inductive absorption)."""
        q = ctrl.voltage_droop(v_measured=1.05, q_setpoint=0.0, droop_pct=5.0)
        assert q < 0.0, "Expected Q < 0 (inductive) for high voltage"

    def test_full_droop_range(self, ctrl):
        """5 % droop: 5 % voltage deviation → rated reactive power."""
        q = ctrl.voltage_droop(
            v_measured=1.0 - 0.05,   # 0.95 pu
            q_setpoint=0.0,
            droop_pct=5.0,
        )
        assert q == pytest.approx(ctrl.rated_kvar, rel=1e-6)

    def test_output_clamped_at_rated_kvar(self, ctrl):
        q = ctrl.voltage_droop(v_measured=0.5, q_setpoint=0.0, droop_pct=5.0)
        assert q <= ctrl.rated_kvar + 1e-9

    def test_output_clamped_at_negative_rated_kvar(self, ctrl):
        q = ctrl.voltage_droop(v_measured=1.5, q_setpoint=0.0, droop_pct=5.0)
        assert q >= -ctrl.rated_kvar - 1e-9

    def test_monotonically_decreasing_with_voltage(self, ctrl):
        """Q must decrease monotonically as voltage increases."""
        voltages = [0.90, 0.95, 1.00, 1.05, 1.10]
        q_vals   = [ctrl.voltage_droop(v, 0.0, droop_pct=5.0) for v in voltages]
        for i in range(len(q_vals) - 1):
            assert q_vals[i] >= q_vals[i + 1] - 1e-9

    def test_setpoint_offset(self, ctrl):
        """Non-zero set-point shifts the characteristic correctly."""
        q_sp = 100.0
        q = ctrl.voltage_droop(v_measured=1.0, q_setpoint=q_sp, droop_pct=5.0)
        assert q == pytest.approx(q_sp, rel=1e-6)

    def test_droop_pct_zero_raises(self, ctrl):
        with pytest.raises(ValueError):
            ctrl.voltage_droop(1.0, 0.0, droop_pct=0.0)


# ---------------------------------------------------------------------------
# Deadband behaviour
# ---------------------------------------------------------------------------

class TestDeadband:
    def test_frequency_within_deadband_no_response(self, ctrl_deadband):
        """Frequency deviation within deadband should produce no droop response."""
        # ±0.05 Hz deadband → 0.03 Hz deviation should not change output
        p_nominal = ctrl_deadband.frequency_droop(60.0,  500.0, droop_pct=5.0)
        p_inside  = ctrl_deadband.frequency_droop(60.03, 500.0, droop_pct=5.0)
        assert p_inside == pytest.approx(p_nominal, abs=1.0)

    def test_frequency_outside_deadband_responds(self, ctrl_deadband):
        """Frequency deviation outside deadband must produce droop response."""
        p_nominal  = ctrl_deadband.frequency_droop(60.0,  500.0, droop_pct=5.0)
        p_outside  = ctrl_deadband.frequency_droop(60.15, 500.0, droop_pct=5.0)
        assert p_outside < p_nominal

    def test_voltage_within_deadband_no_response(self, ctrl_deadband):
        q_nominal = ctrl_deadband.voltage_droop(1.0,    0.0, droop_pct=5.0)
        q_inside  = ctrl_deadband.voltage_droop(1.005,  0.0, droop_pct=5.0)
        assert q_inside == pytest.approx(q_nominal, abs=1.0)


# ---------------------------------------------------------------------------
# Operating point convenience method
# ---------------------------------------------------------------------------

class TestOperatingPoint:
    def test_returns_dict_with_expected_keys(self, ctrl):
        op = ctrl.operating_point(59.8, 0.98, 500.0, 0.0)
        for key in ("f_hz", "v_pu", "p_kw", "q_kvar", "p_load_pct", "q_load_pct"):
            assert key in op, f"Missing key: {key}"

    def test_load_pct_consistent_with_kw(self, ctrl):
        op = ctrl.operating_point(59.8, 0.98, 500.0, 0.0)
        expected_pct = 100.0 * op["p_kw"] / ctrl.rated_kw
        assert op["p_load_pct"] == pytest.approx(expected_pct, rel=1e-6)

    def test_droop_response_captured(self, ctrl):
        """At f < f0, P should be above set-point (governor response)."""
        op = ctrl.operating_point(
            f_measured=59.5, v_measured=1.0, p_setpoint=400.0, q_setpoint=0.0
        )
        assert op["p_kw"] > 400.0


# ---------------------------------------------------------------------------
# Physical / engineering consistency
# ---------------------------------------------------------------------------

class TestPhysicalConsistency:
    def test_p_f_droop_units_correct(self, ctrl):
        """
        P-f droop slope:  Rp = P_rated / (d*f0)  kW/Hz
        At f = f0 - d*f0,  ΔP = P_rated  (from 0 set-point)
        """
        d = 5.0 / 100.0
        f_test = ctrl.rated_freq_hz * (1 - d)
        p = ctrl.frequency_droop(f_test, 0.0, droop_pct=5.0)
        assert p == pytest.approx(ctrl.rated_kw, rel=1e-4)

    def test_q_v_droop_units_correct(self, ctrl):
        """
        Q-V droop slope:  Rq = Q_rated / (d*V0)  kvar/pu
        At V = V0 - d*V0,  ΔQ = Q_rated
        """
        d = 5.0 / 100.0
        v_test = ctrl.rated_voltage_pu * (1 - d)
        q = ctrl.voltage_droop(v_test, 0.0, droop_pct=5.0)
        assert q == pytest.approx(ctrl.rated_kvar, rel=1e-4)

    def test_higher_droop_pct_gives_less_response(self, ctrl):
        """Larger droop % → smaller response for the same frequency deviation."""
        p5  = ctrl.frequency_droop(59.5, 500.0, droop_pct=5.0)
        p10 = ctrl.frequency_droop(59.5, 500.0, droop_pct=10.0)
        assert p5 > p10, "5 % droop should give larger response than 10 %"

    def test_power_output_is_float(self, ctrl):
        p = ctrl.frequency_droop(60.0, 400.0)
        assert isinstance(p, float)
        q = ctrl.voltage_droop(1.0, 0.0)
        assert isinstance(q, float)

    def test_repr_contains_class_name(self, ctrl):
        assert "DroopController" in repr(ctrl)
