# VTOL BEMT Flight-Dynamics MATLAB

MATLAB implementation of an eVTOL / VTOL trim, BEMT rotor, stability, and
control-derivative calculation.

This public version is based directly on the original `BEMTFLAP` calculation
flow. The main equations, trim loop, rotor flapping solve, stability
derivatives, and control-derivative workflow are kept in
`src/BEMTFLAP_SWITCHED.m`. The added layer is a set of switches in `RUN_ME.m`
that choose private lookup data or built-in public defaults.

Private lookup data are not included. With all switches set to `"default"`, the
program can run without a `data/` folder.

## Run

Open MATLAB in this folder and run:

```matlab
RUN_ME
```

or:

```matlab
RUN_TRIM_AND_STABILITY
```

`RUN_TRIM_AND_STABILITY.m` is only a convenience entry point; all settings are
edited in `RUN_ME.m`.

The latest run is saved to:

```text
last_run_results.mat
```

The main output variable is:

```matlab
trim_results
```

It contains:

```matlab
trim_results.trim_table
trim_results.stability_A
trim_results.control_B
trim_results.final_trim_var
trim_results.last_eigenvalues
trim_results.rotor_locations_m
```

## Switches

The switches are at the top of `RUN_ME.m`:

```matlab
data_mode = "default";  % "default" or "lookup"

cfg.switch.geometry = data_mode;
cfg.switch.chord = data_mode;
cfg.switch.pretwist = data_mode;
cfg.switch.airfoil = data_mode;
cfg.switch.fuselage = data_mode;
cfg.switch.controls = data_mode;
```

Use `"default"` for the open-source no-data model. Use `"lookup"` when the
private lookup tables are available locally in `data/`.

## Main Editable Settings

In `RUN_ME.m`:

```matlab
cfg.trim.tilt_angle_deg = 90;
cfg.trim.speed_mps = [0 10 20];
cfg.trim.max_iterations = 10000;
cfg.stability.max_iterations = 10000;
cfg.rotor.airfoil_section_edges = [0.25 0.40 0.50 0.80 0.92];
```

`cfg.rotor.airfoil_section_edges` gives the nondimensional radial cutoff
locations for the six rotor airfoil sections. It must contain five increasing
values between 0 and 1.

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

`rotor_state` is repeated for the six rotors. Its length must be `2*Nb+1`:
first `Nb` flap angles, next `Nb` flap rates, and the last entry is the
induced velocity initial guess. In `RUN_ME.m` it is generated from
`cfg.rotor.blade_count`.

## Lookup Data

If a switch is set to `"lookup"`, place the corresponding private files in a
local `data/` folder next to `RUN_ME.m`.

For a custom location, edit this line in `RUN_ME.m`:

```matlab
data_dir = fullfile(root, 'data');
```

Expected lookup files include:

```text
CS1_cl.txt ... CS6_cl.txt
CS1_cd.txt ... CS6_cd.txt
chord_interp.mat
pretwist_interp.mat
x_cg.mat
z_cg.mat
first_rotor_x.mat
first_rotor_z.mat
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

`model_gpr_uvw.mat` is not required.

## Lookup Table Formats

All text files may be tab- or space-delimited. Angles are in degrees. Each
rectangular grid point should appear exactly once; missing or duplicated grid
points will produce an error when the lookup object is built.

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

Example:

```text
0.01    0.05    0.13    0.30
-180    0       0       0       0
-160    0.647857 0.647857 0.647857 0.647857
```

The first row lists Mach grid points. Every following row starts with
`alpha_deg`; the remaining columns are the coefficient values at each Mach.
The same format is used for both CL and CD. The rotor radial section selection
is set in `RUN_ME.m` by:

```matlab
cfg.rotor.airfoil_section_edges = [0.25 0.40 0.50 0.80 0.92];
```

With the default values, the section mapping is:

```text
x < 0.25  -> CS1
x < 0.40  -> CS2
x < 0.50  -> CS3
x < 0.80  -> CS4
x < 0.92  -> CS5
else      -> CS6
```

### Chord and Pretwist MAT Files

Files:

```text
chord_interp.mat
pretwist_interp.mat
```

`chord_interp.mat` must contain a callable variable named `F`:

```matlab
chord_ratio = F(x);
```

where `x = r/R` is the nondimensional blade radial station. The original code
uses `F(x) * R` as the local chord length.

`pretwist_interp.mat` must contain a callable variable named `pre_twist`:

```matlab
twist_deg = pre_twist(x);
```

The twist value is in degrees and is converted inside the BEMT calculation.

### CG and Front-Rotor Position MAT Files

Files:

```text
x_cg.mat
z_cg.mat
first_rotor_x.mat
first_rotor_z.mat
```

Each file must contain a callable variable with the same name as the file:

```matlab
x_cg_mm          = x_cg(tilt_angle_deg);
z_cg_mm          = z_cg(tilt_angle_deg);
front_rotor_x_mm = first_rotor_x(tilt_angle_deg);
front_rotor_z_mm = first_rotor_z(tilt_angle_deg);
```

The returned values are in millimeters, matching the original `BEMTFLAP`
geometry convention.

### Fuselage/Airframe Coefficient Tables

Files:

```text
Fuselage_cd.txt
Fuselage_cl.txt
Fuselage_cm.txt
Fuselage_cc.txt
Fuselage_cn.txt
Fuselage_cll.txt
```

Format: three columns per row.

```text
tilt_angle_deg   alpha_deg   coefficient
```

Example:

```text
0   -12   0.228091962
0    -9   0.133751702
0    -6   0.073399029
```

The lookup is built as:

```matlab
coefficient = table.CD(tilt_angle_deg, alpha_deg);
```

The file name determines which coefficient is being loaded: `Fuselage_cd.txt`
for CD, `Fuselage_cl.txt` for CL, and so on.

### Control Surface Increment Tables

Files:

```text
Fuselage_elevator.txt
Fuselage_rudder.txt
Fuselage_roll.txt
```

Format: five columns per row.

```text
deflection_deg   alpha_deg   increment_1   increment_2   increment_3
```

For `Fuselage_elevator.txt`, the three increments are:

```text
dCD   dCL   dCM
```

For `Fuselage_rudder.txt` and `Fuselage_roll.txt`, the three increments are
used by the original code as:

```text
dCC   dCN   dCLL
```

Example:

```text
-20   -12   -0.034025137   0.051321743   -0.171615501
-20    -9   -0.013105979   0.142805449   -0.327482425
```

The lookup call returns a row vector:

```matlab
vals = table.eval(deflection_deg, alpha_deg);
```

with the increment order shown above.

## Layout

```text
.
|-- RUN_ME.m
|-- RUN_TRIM_AND_STABILITY.m
|-- README.md
`-- src/
    |-- BEMTFLAP_SWITCHED.m
    |-- CS1_cd_lookup.m
    |-- build_C_lookup_from_txt.m
    `-- build_fuselage_elevator_lookup.m
```

## License

MIT License.
