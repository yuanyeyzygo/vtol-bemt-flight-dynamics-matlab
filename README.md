# VTOL BEMT Flight-Dynamics MATLAB

[Chinese README](README.zh-CN.md)

MATLAB tools for VTOL/eVTOL trim, rotor BEMT, flapping dynamics, stability and control derivatives, and nonlinear response simulation.

The repository is organized around three workflows:

1. **MATLAB individual-blade workflow**: `RUN_ME.m` / `RUN_TRIM_AND_STABILITY.m`. This is the detailed trim, stability-derivative, and control-derivative workflow. It uses an individual-blade flapping model with BEMT, and can run with either public `default` data or user-provided `lookup` / `excel` data.
2. **Simulink individual-blade workflow**: `RUN_RESPONSE_SIMULINK_SETUP.m` with `cfg.rotor.flap_model = "blade"`. This uses the same detailed individual-blade rotor model for nonlinear response in Simulink. It can also use `default`, `lookup`, or `excel` data, but it is computationally slow and is mainly for high-fidelity checks.
3. **Fast Simulink disk-flap workflow**: `RUN_RESPONSE_SIMULINK_SETUP.m` with the default `cfg.rotor.flap_model = "disk"`, followed by `RUN_FAST_DISK_MEX_RESPONSE`. This uses a reduced whole-disk flapping model and MEX acceleration for real-time-style response simulation. This fast workflow is intended for the public `default` model and does not support the full lookup-table path.

Useful references:

- Individual-blade rotor modeling: Stephen Rutherford, *Simulation techniques for the study of the manoeuvring of advanced rotorcraft configurations*, PhD thesis, University of Glasgow, 1997. <https://theses.gla.ac.uk/30844/>
- Disk/tip-path-plane flapping dynamics: R. T. N. Chen, *Effects of primary rotor parameters on flapping dynamics*, NASA TP-1431, 1980. <https://ntrs.nasa.gov/citations/19800006879>

Private aerodynamic lookup data are not included. The code can run in `default` mode without a `data/` folder. Lookup and Excel formats are documented below so users can add their own data.

## Quick Start

Open MATLAB in this folder.

Trim, stability and control derivatives:

```matlab
RUN_ME
```

Nonlinear Simulink/MEX response:

```matlab
RUN_RESPONSE_SIMULINK_SETUP
out = RUN_FAST_DISK_MEX_RESPONSE(2.0);
```

`RUN_RESPONSE_SIMULINK_SETUP` trims the aircraft, exports the current initial conditions to the MATLAB base workspace, rebuilds the fast disk-flap MEX for the current setup, and regenerates the Simulink model.

`RUN_FAST_DISK_MEX_RESPONSE` only runs Simulink. It does not change aerodynamic or flight-dynamics parameters and does not load cached initialization MAT files.

## Main Switches

Each workflow has its own entry script. Where lookup data are supported, the data-source switches use the same names.

### 1. MATLAB Individual-Blade

Use `RUN_ME.m` for trim, stability derivatives, and control derivatives. This workflow uses the detailed individual-blade rotor model.

```matlab
data_mode = "default";          % "default" or "lookup"
aero_database_mode = data_mode; % "default", "lookup", or "excel"

cfg.rotor.flap_model = "blade";

cfg.switch.geometry        = data_mode;
cfg.switch.rotor_positions = data_mode;
cfg.switch.chord           = data_mode;
cfg.switch.pretwist        = data_mode;
cfg.switch.airfoil         = data_mode;
cfg.switch.fuselage        = aero_database_mode;
cfg.switch.controls        = aero_database_mode;
```

`default` runs without private data. `lookup` uses txt lookup tables in `data/`. `excel` is supported for fuselage and control-surface aerodynamics.

### 2. Slow Simulink Individual-Blade

Use `RUN_RESPONSE_SIMULINK_SETUP.m` and select the blade model before setup:

```matlab
flap_model_override = "blade";
RUN_RESPONSE_SIMULINK_SETUP
```

Inside `RUN_RESPONSE_SIMULINK_SETUP.m`, the same data-source switches are used:

```matlab
data_mode = "default";          % "default" or "lookup"
aero_database_mode = data_mode; % "default", "lookup", or "excel"
cfg.rotor.flap_model = requested_flap_model;  % "blade"
```

This path can use `default`, `lookup`, or `excel`, but it is slow because it evaluates the detailed individual-blade response in Simulink.

### 3. Fast Simulink Disk-Flap

Use the default disk model setup and then run the generated fast model:

```matlab
RUN_RESPONSE_SIMULINK_SETUP
out = RUN_FAST_DISK_MEX_RESPONSE(2.0);
```

For this workflow, keep the public default data path:

```matlab
data_mode = "default";
aero_database_mode = "default";
cfg.rotor.flap_model = "disk";
cfg.response.compile_mex = true;
```

The fast MEX path is intended for the numeric default/disk model. It does not support the full lookup-table path. `RUN_FAST_DISK_MEX_RESPONSE` only runs the already generated Simulink/MEX setup; it has no aerodynamic data switches.

## Default Model

The default model is intentionally simple and public-friendly. Important parameters are exposed in `RUN_ME.m` and `RUN_RESPONSE_SIMULINK_SETUP.m`.

Common vehicle and environment parameters:

```matlab
cfg.environment.rho_kg_m3
cfg.environment.gravity_m_s2
cfg.vehicle.mass_kg
cfg.vehicle.inertia_kg_m2   % [Ixx Iyy Izz]
```

Rotor parameters:

```matlab
cfg.rotor.radius_m
cfg.rotor.blade_count
cfg.rotor.omega_rad_s
cfg.rotor.flap_inertia_kg_m2
cfg.rotor.flap_spring_nm_rad
cfg.rotor.rotational_direction
cfg.rotor.blade_element_count
cfg.rotor.azimuth_steps
cfg.rotor.flap_model        % "blade" or "disk"
cfg.rotor.flap_integrator
cfg.rotor.inflow_model
```

Default rotor geometry uses a compact analytic form for each rotor:

```matlab
x = x0 + x_cos*cos(tilt) + x_sin*sin(tilt)
y = y0
z = z0 + z_cos*cos(tilt) + z_sin*sin(tilt)
```

The coefficients are stored in:

```matlab
cfg.defaults.geometry.rotor_position_coeffs_mm
```

Default rotor airfoil and blade planform:

```matlab
cfg.defaults.chord_m
cfg.defaults.pretwist_root_deg
cfg.defaults.pretwist_tip_deg
cfg.defaults.airfoil.cl_alpha_per_rad
cfg.defaults.airfoil.cl_max
cfg.defaults.airfoil.cd0
cfg.defaults.airfoil.cd_alpha2
```

Default fuselage and control parameters:

```matlab
cfg.fuselage.reference_area_m2
cfg.fuselage.mean_aero_chord_m
cfg.fuselage.span_m
cfg.defaults.fuselage.cd0
cfg.defaults.fuselage.cl_alpha_per_rad
cfg.defaults.fuselage.cm_alpha_per_rad
cfg.defaults.fuselage.cc_beta
cfg.defaults.fuselage.cn_beta
cfg.defaults.fuselage.cll_beta
cfg.defaults.controls.elevator
cfg.defaults.controls.rudder
cfg.defaults.controls.aileron
```

Dynamic aerodynamic derivatives are optional and exposed through:

```matlab
cfg.aero.dynamic_derivatives.enabled
cfg.aero.dynamic_derivatives.CLq
cfg.aero.dynamic_derivatives.Cmq
cfg.aero.dynamic_derivatives.Clp
cfg.aero.dynamic_derivatives.Cnp
cfg.aero.dynamic_derivatives.Cyp
cfg.aero.dynamic_derivatives.Cnr
cfg.aero.dynamic_derivatives.Clr
cfg.aero.dynamic_derivatives.Cyr
cfg.aero.dynamic_derivatives.Cma_dot
cfg.aero.dynamic_derivatives.Cn_beta_dot
```

## Trim Outputs

The main result is `trim_results`.

```matlab
trim_results.trim_table
trim_results.stability_A
trim_results.control_B
trim_results.final_trim_var
trim_results.last_eigenvalues
trim_results.rotor_locations_m
trim_results.speed_mps
```

The state order in the linearized `A` matrix is:

```text
[u, w, q, theta, v, p, phi, r]
```

The control order in `B` is:

```text
[collective, longitudinal, lateral, yaw, fixed_pitch, fixed_yaw, fixed_roll]
```

If `cfg.control_blend.enabled = true` and `cfg.control_blend.apply_to_trim = true`, the trim/control variables after collective become blended pilot-equivalent commands:

```text
[collective, blend_pitch, blend_roll, blend_yaw, fixed_pitch, fixed_yaw, fixed_roll]
```

The current default transition schedule is based on nacelle tilt angle:

```matlab
cfg.control_blend.independent_variable = "tilt_angle";
cfg.control_blend.tilt_helicopter_deg = 90;
cfg.control_blend.tilt_fixedwing_deg = 0;
cfg.control_blend.schedule = "sincos";
```

With `schedule = "sincos"`, the code uses the actual tilt angle directly:

```text
tilt_limited = clamp(tilt_angle_deg, min(tilt_helicopter_deg, tilt_fixedwing_deg),
                                     max(tilt_helicopter_deg, tilt_fixedwing_deg))
rotor_weight = clamp(sind(tilt_limited), 0, 1)
fixed_weight = clamp(cosd(tilt_limited), 0, 1)
```

For example, at 90 deg tilt the rotor weight is 1 and fixed-wing weight is 0. At 0 deg tilt the rotor weight is 0 and fixed-wing weight is 1. At 60 deg tilt the weights are `sind(60)` and `cosd(60)`, not a complementary linear pair.

The three blended pilot-equivalent channels are allocated as:

```text
rotor_longitudinal_deg = rotor_weight * rotor_gains(1) * blend_pitch
rotor_lateral_deg      = rotor_weight * rotor_gains(2) * blend_roll
rotor_yaw_deg          = rotor_weight * rotor_gains(3) * blend_yaw

fixed_pitch_deg += fixed_weight * fixed_gains(1) * blend_pitch
fixed_roll_deg  += fixed_weight * fixed_gains(2) * blend_roll
fixed_yaw_deg   += fixed_weight * fixed_gains(3) * blend_yaw
```

Both `rotor_gains` and `fixed_gains` are ordered as `[pitch, roll, yaw]`.

The optional `linear` and `smoothstep` schedules are still supported for older studies. In those modes the code first computes a scalar fixed-wing weight from speed or tilt angle, then uses `rotor_weight = 1 - fixed_weight`.

The fixed-wing command vector is ordered as `[fixed_pitch, fixed_yaw, fixed_roll]` in degrees. When WL/WR/VL/VR physical-surface lookup or Excel sheets are used, these three channels are mapped to physical surface deflections by `cfg.controls.surface_mixing_matrix`. If that matrix is empty, the default mapping is:

```text
gain = cfg.controls.channel_to_physical_gain   % default 0.5

DVL1 = gain * (fixed_yaw - fixed_pitch)
DVR1 = gain * (fixed_yaw + fixed_pitch)
DVL2 = gain * (fixed_yaw - fixed_pitch)
DVR2 = gain * (fixed_yaw + fixed_pitch)

DWL1 = DWR1 = fixed_roll
DWL2 = DWR2 = fixed_roll
```

To use a custom allocator, set:

```matlab
cfg.controls.channel_names = {'pitch','yaw','roll'};
cfg.controls.physical_surface_names = {'WL1','WL2','WR1','WR2','VL1','VL2','VR1','VR2'};
cfg.controls.surface_mixing_matrix = M;  % rows: physical surfaces, columns: channels
cfg.controls.surface_bias_deg = b;       % optional bias for each physical surface
```

## Simulink Response

The Simulink response workflow is:

```matlab
RUN_RESPONSE_SIMULINK_SETUP
out = RUN_FAST_DISK_MEX_RESPONSE(2.0);
```

The setup script is the only place where response sample time and trim condition should be changed:

```matlab
cfg.trim.tilt_angle_deg
cfg.trim.speed_mps
cfg.response.dt_s
cfg.response.control_delta
cfg.response.fixed_wing_control_delta
cfg.response.rotor_tilt_angle_deg
```

`RUN_FAST_DISK_MEX_RESPONSE` has no flight-dynamics parameters. It only runs the current Simulink model using base-workspace variables generated by `RUN_RESPONSE_SIMULINK_SETUP`.

The Simulink model contains a simple control-change interface:

```text
Rotor control delta  = [collective, longitudinal, lateral, yaw]
Fixed-wing control   = [pitch, yaw, roll]
Rotor tilt input     = six nacelle tilt angles in deg
```

The example model uses two Step blocks summed into the collective channel, and constants for the remaining channels. Users can replace these blocks with their own controller outputs.

`out.x_sim` uses the following 12-state order:

```text
[u v w p q r phi theta psi x y z]
```

Coordinate meaning:

```text
u, v, w        body-axis velocities
p, q, r        body-axis angular rates
phi, theta, psi Euler angles
x, y, z        earth/inertial positions
forces_sim     body-axis [Fx Fy Fz Mx My Mz]
```

## Lookup Data

Private lookup data are not included. If lookup mode is enabled, put files in:

```text
data/
```

All txt files must contain numeric values only. Do not add headers, column names, units, comments, or explanatory text. The first line must already be data.

### Rotor Airfoil CL/CD Tables

Files:

```text
CS1_cl.txt ... CS6_cl.txt
CS1_cd.txt ... CS6_cd.txt
```

Format:

```text
Mach_1   Mach_2   Mach_3   ...
alpha_1  value(alpha_1,Mach_1)  value(alpha_1,Mach_2)  value(alpha_1,Mach_3) ...
alpha_2  value(alpha_2,Mach_1)  value(alpha_2,Mach_2)  value(alpha_2,Mach_3) ...
...
```

### Chord and Pretwist

Chord is in meters. Pretwist is in degrees.

```text
r_over_R_1   chord_m_1
r_over_R_2   chord_m_2
...
```

```text
r_over_R_1   pretwist_deg_1
r_over_R_2   pretwist_deg_2
...
```

### CG and Rotor Positions

CG table:

```text
tilt_deg   x_cg_mm   y_cg_mm   z_cg_mm
...
```

Rotor position table:

```text
tilt_deg   r1_x_mm r1_y_mm r1_z_mm   r2_x_mm r2_y_mm r2_z_mm ... r6_x_mm r6_y_mm r6_z_mm
...
```

The code interpolates positions by tilt angle.

### Fuselage Base Tables

Files:

```text
Fuselage_cd.txt
Fuselage_cl.txt
Fuselage_cm.txt
Fuselage_cc.txt
Fuselage_cn.txt
Fuselage_cll.txt
```

Format:

```text
tilt_1   tilt_2   tilt_3   ...
alpha_1  coeff(alpha_1,tilt_1)  coeff(alpha_1,tilt_2)  coeff(alpha_1,tilt_3) ...
alpha_2  coeff(alpha_2,tilt_1)  coeff(alpha_2,tilt_2)  coeff(alpha_2,tilt_3) ...
...
```

For lateral/directional tables such as `cc/cn/cll`, coefficients may represent the value at a reference sideslip or control deflection. Keep the same convention in the code configuration.

### Control Surface Tables

For elevator-like tables:

```text
deflection_deg   alpha_deg   dCD   dCL   dCM
...
```

For rudder/aileron-like tables:

```text
deflection_deg   alpha_deg   dCC   dCN   dCLL
...
```

## Excel Aerodynamic Database

The fuselage and control-surface database can also be stored in one Excel file. MATLAB can read different sheets directly.

A public synthetic template is provided at:

```text
data_templates/aero_database_template.xlsx
```

Copy it to `data/` or point `cfg.data.aero.excel_file` to its path, then replace the placeholder rows with your own data.

Recommended sheets:

```text
base_aero
WL1 WL2 WR1 WR2 VL1 VL2 VR1 VR2
```

For the base fuselage/airframe database, two Excel layouts are supported:

1. Single-sheet layout: keep all nacelle/airframe tilt angles in `base_aero` and include `tilt_angle_deg` as the first numeric column.
2. Split-sheet layout: put each tilt angle in a separate sheet, for example `base_0`, `base_30`, `base_60`, `base_90`. In this case set `cfg.data.aero.base_sheets` and `cfg.data.aero.base_sheet_tilt_angle_deg` in `RUN_ME.m`.

Example split-sheet settings:

```matlab
cfg.switch.fuselage = "excel";
cfg.switch.controls = "excel";
cfg.data.aero.excel_file = 'aero_database.xlsx';
cfg.data.aero.base_sheets = {'base_0','base_30','base_60','base_90'};
cfg.data.aero.base_sheet_tilt_angle_deg = [0 30 60 90];
cfg.data.aero.control_surface_sheets = {'WL1','WL2','WR1','WR2','VL1','VL2','VR1','VR2'};
```

To use the old txt tables instead, set `cfg.switch.fuselage = "lookup"` and `cfg.switch.controls = "lookup"`.

`base_aero` schema:

```text
tilt_angle_deg   beta_deg   alpha_deg   CD   CL   Cm   CC   Cn   Cl
...
```

Split base-sheet schema, when the tilt angle is supplied through `cfg.data.aero.base_sheet_tilt_angle_deg`:

```text
beta_deg   alpha_deg   CD   CL   Cm   CC   Cn   Cl
...
```

Control surface sheet schema:

```text
deflection_deg   alpha_deg   dCD   dCL   dCm   dCC   dCn   dCl
...
```

Older Excel files that still include a `Mach` column in these fuselage/control-surface sheets remain supported, but Mach is not required by the current fuselage/control-surface model.

Excel sheets may include text headers and notes; the MATLAB reader keeps only complete numeric rows. The numeric rows must still form a complete interpolation grid for every independent variable in the sheet.

This repository may include format examples and synthetic placeholder values, but not private aerodynamic data.

## Public Repository Policy

The repository should not include:

```text
data/
legacy/
examples/ that require private data
*.mat result/cache files
*.zip packages
slprj/
codegen_mex*/
*.mexw64
*.slxc
```

Users should generate MEX files locally by running `RUN_RESPONSE_SIMULINK_SETUP`.
