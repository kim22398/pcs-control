"""
pcs — Power Conversion System Control Toolkit
==============================================

A standards-aligned Python library for grid-tied PCS simulation, control, and
protection — targeting BESS PCS, PV inverters, and bidirectional converters.

Public modules
--------------
pcs.converter
    BidirectionalConverter — steady-state model of a 3-level NPC / T-type VSC.
    Includes efficiency curve (calibrated for SiC NPC topology), DC/AC current
    calculations, and SPWM modulation index.

pcs.grid_interface
    GridInterface — point-of-common-coupling (PCC) analysis.
    Includes fault level, Thevenin voltage regulation, and islanding NDZ per
    IEC 62116 Annex B.

pcs.protection
    PCSProtection — IEEE 1547-2018 / IEC 62116 protection functions.
    All methods return (trip: bool, reason: str).

pcs.control.droop
    DroopController — P-f and Q-V droop for grid-forming PCS.

pcs.control.grid_following
    PLLController  — SRF-PLL for grid synchronisation.
    CurrentController — PI inner current controller in dq frame.

pcs.control.mppt
    MPPTController — Perturb & Observe MPPT for PV front-ends.

Standards
---------
- IEEE 1547-2018 — DER interconnection and interoperability.
- IEC 62116:2014 — Islanding prevention test procedure.
- IEC 61727:2004 — PV grid interface characteristics.
- NERC PRC-024-3 — Frequency/voltage relay settings reference.
"""
