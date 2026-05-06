# VTOL BEMT Flight-Dynamics MATLAB

Clean MATLAB interface for an eVTOL / VTOL BEMT flight-dynamics model.

This public version is intentionally lightweight:

- no bundled private lookup data
- no legacy original script
- no required examples
- default analytic rotor and airframe models run without a `data/` folder
- includes a clean public trim/stability/control-derivative workflow

The main user interface is:

```matlab
RUN_ME
```

By default, `RUN_ME` uses the no-lookup analytic model profile. This means the
repository can be cloned or downloaded and run without any lookup tables.

## Main Inputs

Most settings are edited in `RUN_ME.m`.

The tilt angle is defined once near the top:

```matlab
tilt_angle_deg = 60.0;
```

Rotor geometry and installation are defined through:

```matlab
rotor_count
rotor_geometry_model
rotor_positions_m
rotational_direction
rotor_template.R
rotor_template.Nb
rotor_template.omega
rotor_template.tilt_angle_deg
```

By default, the public no-data profile uses `rotor_geometry_model =
'legacy_default'`. This follows the original `BEMTFLAP.m`
`rotor_locations()` / `local_transform()` coordinate convention, with built-in
scalar fallback values so the package runs without private geometry tables.

Lookup mode uses `rotor_geometry_model = 'legacy_lookup'`, which reads the
original CG and rotor geometry tables and produces the same local rotor
locations as `BEMTFLAP.m`.

The default public profile uses:

```matlab
rotor_template.airfoil_model  = 'linear';
rotor_template.chord_model    = 'linear';
rotor_template.pretwist_model = 'linear';
airframe.model                = 'component_bem';
```

## Optional Lookup Mode

The code still supports lookup data if you add your own tables locally. To use
the lookup profile, run:

```matlab
RUN_ME("run", "user")
```

Then provide a local `data/` folder with the expected CL/CD, chord, pretwist,
and airframe lookup files.

For compatibility with the original `BEMTFLAP.m` workflow, lookup mode also
expects these local geometry tables when `rotor_geometry_model =
'legacy_lookup'` in `RUN_ME.m`:

```text
x_cg.mat
z_cg.mat
first_rotor_x.mat
first_rotor_z.mat
```

The public zip does not include those private geometry/data files.

## Trim Initial Values

Trim initial values are collected in `RUN_ME.m` under `options.trim.initial`:

```matlab
options.trim.initial.beta_rad
options.trim.initial.beta_dot_rad
options.trim.initial.induced_velocity
options.trim.initial.collective_deg
options.trim.initial.longitudinal_deg
options.trim.initial.lateral_deg
options.trim.initial.yaw_deg
options.trim.initial.pitch_rad
options.trim.initial.roll_rad
```

For stability derivatives, rotor flapping and induced-velocity states are
re-solved after each perturbation by default:

```matlab
options.trim.retrim_rotor_states_for_derivatives = true;
options.trim.stability_max_iter = 15;
```

This matches the intent of the original workflow, where each perturbed
condition first re-established rotor periodic states before computing force
and moment derivatives.

Trim and linearization can be run directly:

```matlab
trim_results = RUN_TRIM_AND_STABILITY
```

Or from `RUN_ME.m`:

```matlab
run_trim_and_stability = true;
```

The public trim workflow returns the trim table, stacked stability matrices,
rotor-control derivatives, and elevator/rudder/aileron control derivatives:

```matlab
trim_results.Mttt
trim_results.MMA
trim_results.MMB
trim_results.A
trim_results.B_all
trim_results.derivatives
trim_results.fixed_surface_derivatives
```

This is a cleaned implementation of the trim/stability path. It uses the public
rotor and airframe switches and does not require the original legacy script.

## Layout

```text
.
|-- RUN_ME.m
|-- RUN_TRIM_AND_STABILITY.m
|-- README.md
`-- src/
    |-- evtol_run_trim_stability.m
    |-- evtol_configure_vehicle_models.m
    |-- evtol_airframe_forces.m
    |-- evtol_default_config.m
    |-- evtol_load_model_data.m
    |-- evtol_make_legacy_inputs.m
    |-- evtol_rotor_locations.m
    |-- build_C_lookup_from_txt.m
    `-- build_fuselage_elevator_lookup.m
```

## License

MIT License.
