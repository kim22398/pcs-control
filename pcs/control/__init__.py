"""
pcs.control — PCS Control Algorithms Sub-Package
=================================================

Provides grid-forming and grid-following control algorithms for voltage-source
converters (VSC) in grid-tied power conversion systems.

Modules
-------
droop
    DroopController — stateless P-f and Q-V droop for grid-forming PCS.
    Implements IEEE 1547.4 / CIGRE WG C6.22 droop characteristics with
    optional deadband and output clamping.

grid_following
    PLLController    — Synchronous Reference Frame PLL (SRF-PLL).
                       Tracks grid voltage angle by driving vq → 0.
    CurrentController — PI inner current controller in the dq rotating frame.
                        Includes cross-coupling decoupling, voltage feed-forward,
                        and conditional-integration anti-windup.

mppt
    MPPTController — Perturb & Observe (P&O) MPPT for PV front-ends.
                     Returns voltage reference for DC/DC or outer PCS loop.

Control hierarchy (grid-forming BESS PCS)
-----------------------------------------
EMS dispatch (P*, Q*)
    → DroopController (outer: P-f and Q-V droop)
        → CurrentController (inner: dq PI current control)
            → Modulator (SPWM/SVPWM → gate signals)
    || PCSProtection (supervisory, parallel)

Control hierarchy (grid-following PV inverter)
-----------------------------------------------
MPPTController (outermost: maximise PV harvest)
    → PLLController (grid synchronisation: track θ)
        → CurrentController (inner: dq PI current control)
            → Modulator
    || PCSProtection (supervisory, parallel)
"""
