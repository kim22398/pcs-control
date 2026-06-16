"""
main.py
=======
Command-line entry point for the PCS Control Toolkit.

This thin CLI wraps the :mod:`pcs` library so the project can be driven without
writing any Python.  It exposes the flagship BESS PCS demo plus a handful of
domain sub-commands that compute the library's core quantities directly.

Usage
-----
    python main.py                 # run the flagship BESS PCS droop demo
    python main.py --help          # list all sub-commands
    python main.py test            # run the pytest suite
    python main.py simulate        # parametrised P-f / Q-V droop sweep
    python main.py droop  --freq 59.7 --p-setpoint-kw 400 --droop-pct 5
    python main.py converter --power-kw 500 --ac-voltage 690 --dc-voltage 1150

All sub-commands ship sensible defaults, so each one runs with no arguments.

Notes
-----
The ``sys.path`` insert at the top lets ``import pcs`` resolve when the file is
run directly as ``python main.py`` from any directory, without needing to set
``PYTHONPATH`` or install the package.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import argparse
import math
import runpy

from pcs.converter import BidirectionalConverter
from pcs.control.droop import DroopController
from pcs.protection import PCSProtection

# Repo-root absolute paths so sub-commands work from any working directory.
_REPO_ROOT = os.path.dirname(os.path.abspath(__file__))
_DEMO_PATH = os.path.join(_REPO_ROOT, "examples", "bess_pcs_demo.py")
_TESTS_DIR = os.path.join(_REPO_ROOT, "tests")


# ---------------------------------------------------------------------------
# Sub-command handlers
# ---------------------------------------------------------------------------

def cmd_demo(args: argparse.Namespace) -> int:
    """Run the flagship BESS PCS droop demo (``examples/bess_pcs_demo.py``)."""
    runpy.run_path(_DEMO_PATH, run_name="__main__")
    return 0


def cmd_test(args: argparse.Namespace) -> int:
    """Run the pytest suite via ``python -m pytest tests/ -q``."""
    import subprocess

    return subprocess.call(
        [sys.executable, "-m", "pytest", _TESTS_DIR, "-q"],
        cwd=_REPO_ROOT,
    )


def cmd_droop(args: argparse.Namespace) -> int:
    """
    Compute a single P-f / Q-V droop operating point and print it.

    Exposes :meth:`DroopController.operating_point` so the active/reactive power
    response to a given frequency and voltage can be evaluated from the shell.
    """
    ctrl = DroopController(
        rated_kw=args.rated_kw,
        rated_kvar=args.rated_kvar,
        rated_freq_hz=args.rated_freq,
        rated_voltage_pu=1.0,
    )
    op = ctrl.operating_point(
        f_measured=args.freq,
        v_measured=args.voltage,
        p_setpoint=args.p_setpoint_kw,
        q_setpoint=args.q_setpoint_kvar,
        p_droop_pct=args.droop_pct,
        q_droop_pct=args.droop_pct,
    )

    print()
    print("Droop operating point")
    print("=" * 52)
    print(f"  Rated power           : {ctrl.rated_kw:>10.1f} kW / "
          f"{ctrl.rated_kvar:.1f} kvar")
    print(f"  Nominal frequency     : {ctrl.rated_freq_hz:>10.3f} Hz")
    print(f"  Droop coefficient     : {args.droop_pct:>10.2f} %")
    print("-" * 52)
    print(f"  Measured frequency    : {op['f_hz']:>10.3f} Hz "
          f"({op['f_hz'] - ctrl.rated_freq_hz:+.3f} Hz)")
    print(f"  Measured voltage      : {op['v_pu']:>10.3f} pu")
    print("-" * 52)
    print(f"  Active power  P_out   : {op['p_kw']:>10.1f} kW   "
          f"({op['p_load_pct']:+.1f} % of rated)")
    print(f"  Reactive power Q_out  : {op['q_kvar']:>10.1f} kvar "
          f"({op['q_load_pct']:+.1f} % of rated)")
    print("=" * 52)
    return 0


def cmd_converter(args: argparse.Namespace) -> int:
    """
    Compute steady-state converter metrics for a given operating power.

    Exposes :class:`BidirectionalConverter` — efficiency, DC/AC currents, and
    the SPWM modulation index — so the DC-bus sizing can be sanity-checked
    (``m <= 1.0`` for linear modulation) directly from the shell.
    """
    conv = BidirectionalConverter(
        rated_kw=args.rated_kw,
        dc_voltage_V=args.dc_voltage,
        ac_voltage_V=args.ac_voltage,
        switching_freq_hz=args.switching_freq,
    )

    power_kw = args.power_kw
    load_factor = power_kw / conv.rated_kw
    eta = conv.efficiency(load_factor)
    i_dc = conv.dc_current(power_kw)
    i_ac = conv.ac_current_A(power_kw, pf=args.pf)
    m_a = conv.modulation_index()
    mod_state = "linear" if m_a <= 1.0 else "OVER-MODULATION (m > 1.0)"

    print()
    print("Converter operating point")
    print("=" * 52)
    print(f"  Rated power           : {conv.rated_kw:>10.1f} kW")
    print(f"  DC-link voltage       : {conv.dc_voltage_V:>10.1f} V")
    print(f"  AC voltage (LL, RMS)  : {conv.ac_voltage_V:>10.1f} V")
    print(f"  Switching frequency   : {conv.switching_freq_hz:>10.0f} Hz")
    print("-" * 52)
    print(f"  Output power          : {power_kw:>10.1f} kW   "
          f"({load_factor * 100:.1f} % load)")
    print(f"  Power factor          : {args.pf:>10.3f}")
    print("-" * 52)
    print(f"  Efficiency  η         : {eta * 100:>10.2f} %")
    print(f"  DC current  I_dc      : {i_dc:>10.1f} A")
    print(f"  AC current  I_ac      : {i_ac:>10.1f} A")
    print(f"  Modulation index m_a  : {m_a:>10.4f}  [{mod_state}]")
    print("=" * 52)
    return 0


def cmd_simulate(args: argparse.Namespace) -> int:
    """
    Run a parametrised P-f / Q-V droop sweep across a frequency-dip event.

    Unlike the flagship demo (which renders a four-panel PNG), this prints a
    compact text table of the droop response and the converter loading as the
    grid frequency and voltage are swept linearly between two endpoints — handy
    for quick what-if studies and for piping into other tools.
    """
    ctrl = DroopController(
        rated_kw=args.rated_kw,
        rated_kvar=args.rated_kvar,
        rated_freq_hz=args.rated_freq,
        rated_voltage_pu=1.0,
    )
    conv = BidirectionalConverter(
        rated_kw=args.rated_kw,
        dc_voltage_V=args.dc_voltage,
        ac_voltage_V=args.ac_voltage,
    )
    prot = PCSProtection()

    steps = max(2, args.steps)

    print()
    print("=" * 78)
    print(f"  Droop sweep — {ctrl.rated_kw / 1000:.2f} MW / "
          f"{ctrl.rated_kvar / 1000:.2f} MVAr  "
          f"({args.droop_pct:.0f} % droop)")
    print("=" * 78)
    print(f"{'f (Hz)':>9} {'V (pu)':>8} {'P (kW)':>10} {'Q (kvar)':>10} "
          f"{'η (%)':>8} {'I_ac (A)':>10} {'Trip':>6}")
    print("-" * 78)

    for i in range(steps):
        frac = i / (steps - 1)
        f_hz = args.f_start + (args.f_end - args.f_start) * frac
        v_pu = args.v_start + (args.v_end - args.v_start) * frac

        p_kw = ctrl.frequency_droop(f_hz, args.p_setpoint_kw, args.droop_pct)
        q_kvar = ctrl.voltage_droop(v_pu, args.q_setpoint_kvar, args.droop_pct)

        load_factor = p_kw / ctrl.rated_kw if ctrl.rated_kw > 0 else 0.0
        eta = conv.efficiency(load_factor) if load_factor > 0 else float("nan")

        s_kva = math.sqrt(p_kw ** 2 + q_kvar ** 2)
        pf = p_kw / s_kva if s_kva > 0 else 1.0
        i_ac = conv.ac_current_A(p_kw, max(pf, 0.01)) if p_kw > 0 else 0.0

        result = prot.evaluate_all(
            v_pu=v_pu, f_hz=f_hz,
            v_dc=conv.dc_voltage_V, v_dc_max=args.dc_voltage * 1.13,
            i_meas=i_ac, i_max=args.i_max,
        )
        trip = "TRIP" if result["any_trip"] else "OK"
        eta_pct = eta * 100 if not math.isnan(eta) else 0.0

        print(f"{f_hz:9.3f} {v_pu:8.3f} {p_kw:10.1f} {q_kvar:10.1f} "
              f"{eta_pct:8.2f} {i_ac:10.1f} {trip:>6}")

    print("=" * 78)
    print(f"  Modulation index m_a = {conv.modulation_index():.4f} "
          f"({'linear' if conv.modulation_index() <= 1.0 else 'OVER-MODULATION'})")
    print()
    return 0


# ---------------------------------------------------------------------------
# Argument parser
# ---------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    """Construct the top-level argparse parser with all sub-commands."""
    parser = argparse.ArgumentParser(
        prog="main.py",
        description=(
            "PCS Control Toolkit CLI — simulate and analyse grid-tied BESS / "
            "PV power-conversion systems. Run with no sub-command to launch "
            "the flagship BESS PCS droop demo."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.set_defaults(func=cmd_demo)

    sub = parser.add_subparsers(
        title="sub-commands",
        metavar="{demo,test,simulate,droop,converter}",
    )

    # --- demo -------------------------------------------------------------
    p_demo = sub.add_parser(
        "demo",
        help="run the flagship BESS PCS droop demo (default action)",
        description="Run examples/bess_pcs_demo.py (1 MW / 2 MWh BESS, 60 s).",
    )
    p_demo.set_defaults(func=cmd_demo)

    # --- test -------------------------------------------------------------
    p_test = sub.add_parser(
        "test",
        help="run the pytest suite (python -m pytest tests/ -q)",
        description="Shell out to pytest and run the full test suite.",
    )
    p_test.set_defaults(func=cmd_test)

    # --- simulate ---------------------------------------------------------
    p_sim = sub.add_parser(
        "simulate",
        help="run a parametrised P-f / Q-V droop sweep (text table)",
        description="Sweep grid frequency and voltage and tabulate the droop "
                    "response and converter loading.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p_sim.add_argument("--rated-kw", type=float, default=1_000.0,
                       help="rated active power (kW)")
    p_sim.add_argument("--rated-kvar", type=float, default=600.0,
                       help="rated reactive power (kvar)")
    p_sim.add_argument("--rated-freq", type=float, default=60.0,
                       help="nominal grid frequency (Hz)")
    p_sim.add_argument("--ac-voltage", type=float, default=690.0,
                       help="AC line-line RMS voltage (V)")
    p_sim.add_argument("--dc-voltage", type=float, default=1_150.0,
                       help="DC-link voltage (V); keep m_a <= 1.0")
    p_sim.add_argument("--p-setpoint-kw", type=float, default=400.0,
                       help="active power set-point at f0 (kW)")
    p_sim.add_argument("--q-setpoint-kvar", type=float, default=0.0,
                       help="reactive power set-point at V0 (kvar)")
    p_sim.add_argument("--droop-pct", type=float, default=5.0,
                       help="P-f / Q-V droop coefficient (%%)")
    p_sim.add_argument("--f-start", type=float, default=60.0,
                       help="frequency at start of sweep (Hz)")
    p_sim.add_argument("--f-end", type=float, default=59.5,
                       help="frequency at end of sweep (Hz)")
    p_sim.add_argument("--v-start", type=float, default=1.00,
                       help="voltage at start of sweep (pu)")
    p_sim.add_argument("--v-end", type=float, default=0.96,
                       help="voltage at end of sweep (pu)")
    p_sim.add_argument("--steps", type=int, default=11,
                       help="number of sweep points")
    p_sim.add_argument("--i-max", type=float, default=1_200.0,
                       help="AC overcurrent trip threshold (A)")
    p_sim.set_defaults(func=cmd_simulate)

    # --- droop ------------------------------------------------------------
    p_droop = sub.add_parser(
        "droop",
        help="compute a single P-f / Q-V droop operating point",
        description="Evaluate the active/reactive power droop response to a "
                    "measured frequency and voltage.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p_droop.add_argument("--freq", type=float, default=59.7,
                         help="measured grid frequency (Hz)")
    p_droop.add_argument("--voltage", type=float, default=0.98,
                         help="measured PCC voltage (pu)")
    p_droop.add_argument("--p-setpoint-kw", type=float, default=400.0,
                         help="active power set-point at f0 (kW)")
    p_droop.add_argument("--q-setpoint-kvar", type=float, default=0.0,
                         help="reactive power set-point at V0 (kvar)")
    p_droop.add_argument("--droop-pct", type=float, default=5.0,
                         help="droop coefficient (%%)")
    p_droop.add_argument("--rated-kw", type=float, default=1_000.0,
                         help="rated active power (kW)")
    p_droop.add_argument("--rated-kvar", type=float, default=600.0,
                         help="rated reactive power (kvar)")
    p_droop.add_argument("--rated-freq", type=float, default=60.0,
                         help="nominal grid frequency (Hz)")
    p_droop.set_defaults(func=cmd_droop)

    # --- converter --------------------------------------------------------
    p_conv = sub.add_parser(
        "converter",
        help="compute converter efficiency, currents, and modulation index",
        description="Evaluate steady-state converter metrics and verify the "
                    "DC-bus gives linear modulation (m_a <= 1.0).",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p_conv.add_argument("--power-kw", type=float, default=500.0,
                        help="output active power (kW)")
    p_conv.add_argument("--pf", type=float, default=1.0,
                        help="displacement power factor in (0, 1]")
    p_conv.add_argument("--rated-kw", type=float, default=1_000.0,
                        help="rated active power (kW)")
    p_conv.add_argument("--dc-voltage", type=float, default=1_150.0,
                        help="DC-link voltage (V)")
    p_conv.add_argument("--ac-voltage", type=float, default=690.0,
                        help="AC line-line RMS voltage (V)")
    p_conv.add_argument("--switching-freq", type=float, default=10_000.0,
                        help="PWM switching frequency (Hz)")
    p_conv.set_defaults(func=cmd_converter)

    return parser


def main(argv: list[str] | None = None) -> int:
    """Parse arguments and dispatch to the selected sub-command."""
    parser = build_parser()
    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
