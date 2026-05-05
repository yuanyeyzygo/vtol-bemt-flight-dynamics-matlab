# VTOL BEMT Flight-Dynamics MATLAB

Clean MATLAB interface for an eVTOL / VTOL BEMT flight-dynamics model.

This public version is intentionally lightweight:

- no bundled private lookup data
- no legacy original script
- no required examples
- default analytic rotor and airframe models run without a `data/` folder

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
rotor_positions_m
rotational_direction
rotor_template.R
rotor_template.Nb
rotor_template.omega
rotor_template.tilt_angle_deg
```

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

The public repository does not include the original legacy trim/stability
solver. That solver depended on the private original program and data. The
clean public code keeps the configuration, model loading, rotor/airframe
switches, and default force-model path ready for further open refactoring.

## Layout

```text
.
|-- RUN_ME.m
|-- README.md
`-- src/
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
