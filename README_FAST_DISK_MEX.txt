Fast default disk-flap MEX response package
===========================================

Purpose
-------
This is the fast Simulink response version:
- default/no-lookup data path
- updated default geometry/aero parameters
- rotor radius R = 1.3 m
- flap stiffness k_beta = 16000 N*m/rad
- disk flap model
- RK4 aircraft update
- RK4 disk-state update
- disk_state_substeps = 8
- sample time dt_sim = 0.01 s
- compiled MEX step: VTOL_RESPONSE_SIM_STEP_disk_fast_mex.mexw64

How to run
----------
In MATLAB, open this folder and run:

    RUN_RESPONSE_SIMULINK_SETUP
    out = RUN_FAST_DISK_MEX_RESPONSE(2.0);

Main outputs are in the SimulationOutput object:
- out.x_sim
- out.rotor_state_sim
- out.forces_sim
- out.vi_sim
- out.vi_qs_sim
- out.tau_sim

Important notes
---------------
Run RUN_RESPONSE_SIMULINK_SETUP first. It trims the current case, rebuilds the
fast disk MEX for the current setup, and exports:

    sim_init, x0, rotor_state0, control_delta, rotor_tilt_angle_deg,
    fixed_control_delta, dt_sim, sim_model

The runner uses those current base-workspace variables. It does not load
cached initialization MAT files.

If model parameters are changed, run RUN_RESPONSE_SIMULINK_SETUP again before
running Simulink. BUILD_FAST_DISK_MEX is only a convenience wrapper:

    BUILD_FAST_DISK_MEX

This package intentionally does not include lookup-table data.
