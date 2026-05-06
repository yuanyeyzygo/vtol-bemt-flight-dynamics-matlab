# VTOL BEMT Flight-Dynamics MATLAB

MATLAB implementation of a VTOL/eVTOL trim, rotor BEMT, flapping, stability,
and control-derivative calculation.

The public code can run without private lookup data. Set the switches in
`RUN_ME.m` to `"default"` for built-in default models, or to `"lookup"` to read
local text lookup tables from `data/`.

## Run

Open MATLAB in this folder and run:

```matlab
RUN_ME
```

`RUN_TRIM_AND_STABILITY.m` is a convenience wrapper that calls `RUN_ME.m`.

The latest result is saved to:

```text
last_run_results.mat
```

The main output variable is `trim_results`, including:

```matlab
trim_results.trim_table
trim_results.stability_A
trim_results.control_B
trim_results.final_trim_var
trim_results.last_eigenvalues
trim_results.rotor_locations_m
trim_results.speed_mps
trim_results.n_speed_points
```

## Main Interface

Edit `RUN_ME.m`.

```matlab
data_mode = "default";  % "default" or "lookup"

cfg.switch.geometry = data_mode;
cfg.switch.rotor_positions = data_mode;
cfg.switch.chord = data_mode;
cfg.switch.pretwist = data_mode;
cfg.switch.airfoil = data_mode;
cfg.switch.fuselage = data_mode;
cfg.switch.controls = data_mode;
```

Use `"default"` to run without a `data/` folder. Use `"lookup"` when the
corresponding text files are available in `data/`.

Common trim and rotor settings:

```matlab
cfg.trim.tilt_angle_deg = 90;       % common airframe/CG/fuselage lookup angle
cfg.rotor.tilt_angle_deg = 90;      % scalar common nacelle tilt
% cfg.rotor.tilt_angle_deg = [90 90 90 90 85 85]; % per-rotor nacelle tilts
cfg.trim.speed_mps = [0 10 20];
cfg.trim.use_previous_solution = true;
cfg.trim.max_iterations = 10000;
cfg.stability.max_iterations = 10000;
cfg.rotor.blade_count = 5;
cfg.rotor.airfoil_section_edges = [0.25 0.40 0.50 0.80 0.92];
```

When `cfg.trim.use_previous_solution = true`, the first speed uses the initial
guesses in `RUN_ME.m`, and each later speed starts from the previous converged
trim vector. This is usually faster for speed sweeps.

`cfg.rotor.airfoil_section_edges` contains five nondimensional radial cutoff
locations for the six rotor airfoil sections.

Trim initial guesses are also in `RUN_ME.m`:

```matlab
cfg.trim.initial.rotor_state
cfg.trim.initial.collective_deg
cfg.trim.initial.longitudinal_deg
cfg.trim.initial.lateral_deg
cfg.trim.initial.yaw_deg
cfg.trim.initial.pitch_rad
cfg.trim.initial.roll_rad
```

`rotor_state` is repeated for all six rotors. Its length must be `2*Nb+1`:
first `Nb` flap angles, next `Nb` flap rates, and the final entry is induced
velocity.

## Lookup Data Folder

Private lookup data are not included in this repository. If lookup switches
are enabled, place text files in:

```text
data/
```

or edit this line in `RUN_ME.m`:

```matlab
data_dir = fullfile(root, 'data');
```

Expected text files:

```text
CS1_cl.txt ... CS6_cl.txt
CS1_cd.txt ... CS6_cd.txt
Chord.txt
Pretwist.txt
CG_positions.txt
Rotor_positions.txt
Fuselage_cd.txt
Fuselage_cl.txt
Fuselage_cm.txt
Fuselage_cc.txt
Fuselage_cn.txt
Fuselage_cll.txt
Fuselage_elevator.txt
Fuselage_rudder.txt
Fuselage_roll.txt
```

All text files may be space- or tab-delimited. Angles are in degrees unless
noted otherwise.

## Rotor Airfoil CL/CD Tables

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

Example:

```text
0.01    0.05    0.13    0.30
-180    0       0       0       0
-160    0.647857 0.647857 0.647857 0.647857
```

The first row is the Mach grid. Each following row starts with `alpha_deg`;
the remaining columns are coefficient values at the Mach grid points. Do not
add a text header row.

The section mapping is controlled by:

```matlab
cfg.rotor.airfoil_section_edges = [0.25 0.40 0.50 0.80 0.92];
```

With the default values:

```text
x < 0.25  -> CS1
x < 0.40  -> CS2
x < 0.50  -> CS3
x < 0.80  -> CS4
x < 0.92  -> CS5
else      -> CS6
```

## Chord and Pretwist Tables

Files:

```text
Chord.txt
Pretwist.txt
```

`Chord.txt` has two numeric columns:

```text
r_over_R  chord_m
```

`Pretwist.txt` has two numeric columns:

```text
r_over_R  twist_deg
```

`r_over_R = r/R`. Chord is stored directly in metres. If the source gives
nondimensional `c/R`, multiply by rotor radius before writing `Chord.txt`.
The BEMT calculation uses `chord_m * dr` directly.

## CG Position Table

File:

```text
CG_positions.txt
```

Format:

```text
tilt_angle_deg  x_cg_mm  y_cg_mm  z_cg_mm
```

The values are absolute aircraft coordinates in millimetres and are
interpolated by common airframe tilt angle.

## Rotor Position Table

File:

```text
Rotor_positions.txt
```

Format:

```text
rotor_id  tilt_angle_deg  x_mm  y_mm  z_mm
```

There must be one row for each rotor at each available tilt angle. The
coordinates are absolute aircraft coordinates in millimetres. The code converts
them to CG-relative body coordinates internally before calling the rotor BEMT
routine.

`cfg.rotor.tilt_angle_deg` may be scalar or a six-element vector. A scalar is
applied to all rotors; a vector gives individual nacelle tilt angles.

## Fuselage/Airframe Coefficient Tables

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
tilt_angle_deg  alpha_deg  coefficient
```

The file name determines the coefficient. For example,
`Fuselage_cd.txt` supplies CD and `Fuselage_cm.txt` supplies CM.

## Control Surface Increment Tables

Files:

```text
Fuselage_elevator.txt
Fuselage_rudder.txt
Fuselage_roll.txt
```

Format:

```text
deflection_deg  alpha_deg  increment_1  increment_2  increment_3
```

For `Fuselage_elevator.txt`, the increments are:

```text
dCD  dCL  dCM
```

For `Fuselage_rudder.txt` and `Fuselage_roll.txt`, the increments are:

```text
dCC  dCN  dCLL
```

The lookup call returns the three increments in the order above.

## Repository Layout

```text
.
|-- RUN_ME.m
|-- RUN_TRIM_AND_STABILITY.m
|-- README.md
`-- src/
    |-- BEMTFLAP_SWITCHED.m
    |-- CS1_cd_lookup.m
    |-- build_1d_lookup_from_txt.m
    |-- build_cg_lookup_from_txt.m
    |-- build_rotor_position_lookup_from_txt.m
    |-- build_C_lookup_from_txt.m
    `-- build_fuselage_elevator_lookup.m
```

## License

MIT License.
