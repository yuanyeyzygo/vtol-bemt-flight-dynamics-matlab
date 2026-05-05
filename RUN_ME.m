function [state, vehicle, cfg, model, legacy, options] = RUN_ME(mode, model_profile)
%RUN_ME Main user interface for the eVTOL BEMT/flight-dynamics package.
%   Open this file in MATLAB and press Run. The input layout intentionally
%   follows the single-rotor-bemt-matlab style:
%
%     state
%     vehicle.rotors(i)
%     vehicle.airframe
%     options
%
%   The rotor model switches and airframe model switch are kept in this file.

if nargin < 1
    mode = "run";
end
if nargin < 2
    model_profile = "default";
end
mode = string(mode);
model_profile = string(model_profile);
configure_only = any(strcmpi(mode, ["config_only", "configure_only"]));
use_all_default_models = any(strcmpi(model_profile, ["default", "defaults", "no_lookup", "analytic"]));

root = fileparts(mfilename("fullpath"));
addpath(fullfile(root, "src"));

%% ========================= USER SETTINGS =========================
data_dir = fullfile(root, "data");   % or use an absolute path, e.g. "D:\No_Copy"
tilt_angle_deg = 60.0;               % single source for rotor, airframe lookup, and trim

% Runtime checks. The public no-data profile does not need example scripts.
run_data_validation = false;
run_lookup_regression = false;
run_trim_and_stability = false;  % Full original trim/stability sweep. Can be slow.
show_config_preview = true;

if use_all_default_models
    run_data_validation = false;
    run_lookup_regression = false;
end

%% State definition, same spirit as single-rotor-bemt-matlab.
state = struct();
state.velo_body = [20; 5; 0];
state.angular_velocity_body = [0; 0; 0];
state.acceleration_body = [0; 0; 0];
state.angular_acceleration_body = [0; 0; 0];

state.control = struct();
state.control.elevator_deg = 0;
state.control.rudder_deg = 0;
state.control.aileron_deg = 0;

%% Vehicle constants.
vehicle = struct();
vehicle.rho = 1.225;
vehicle.g = 9.81;
vehicle.mass_kg = 1900;
vehicle.inertia_kgm2 = diag([1966.5, 5245.3, 3282.7]);
vehicle.cg_m = [0; 0; 0];

%% Rotor array. Existing fields match the single-rotor repository where possible.
rotor_count = 6;
rotor_positions_m = [ ...
    2.377,  6.000, -1.999; ...
    1.900,  2.500, -1.850; ...
    1.900, -2.500, -1.850; ...
    2.377, -6.000, -1.999; ...
    4.754,  2.500, -3.099; ...
    4.754, -2.500, -3.099];
rotational_direction = [1, -1, 1, -1, 1, -1];

% Rotor model selectors:
%   airfoil_model  : 'linear' or 'c81txt'
%   chord_model    : 'linear' or 'lookup'
%   pretwist_model : 'linear' or 'lookup'
rotor_template = struct();
rotor_template.R = 1.5;
rotor_template.Nb = 5;
rotor_template.omega = 83.775804;
rotor_template.rho = vehicle.rho;
rotor_template.I_beta = 2.25;
rotor_template.k_beta = 16000;
rotor_template.tilt_angle_deg = tilt_angle_deg;
rotor_template.rotational_direction = 1;
rotor_template.hub_loc = [0; 0; 0];
rotor_template.root_cutout = 0.15;
rotor_template.n_be = 10;
rotor_template.n_az = 72;
rotor_template.theta0_deg = 18;

if use_all_default_models
    rotor_template.airfoil_model = 'linear';
    rotor_template.chord_model = 'linear';
    rotor_template.pretwist_model = 'linear';
else
    rotor_template.airfoil_model = 'c81txt';
    rotor_template.chord_model = 'lookup';
    rotor_template.pretwist_model = 'lookup';
end

rotor_template.section_r_end = [0.25, 0.40, 0.50, 0.80, 0.92, 1.00];
rotor_template.section_airfoil_id = [1, 2, 3, 4, 5, 6];

switch lower(rotor_template.airfoil_model)
    case 'linear'
        rotor_template.airfoil_linear = struct( ...
            'a0', 5.7, ...
            'alpha0_deg', 0.0, ...
            'cd0', 0.01, ...
            'k', 0.02);

    case 'c81txt'
        rotor_template.c81_pairs = repmat(struct('cl_file', '', 'cd_file', ''), 1, 6);
        rotor_template.c81_pairs(1).cl_file = 'CS1_cl.txt'; rotor_template.c81_pairs(1).cd_file = 'CS1_cd.txt';
        rotor_template.c81_pairs(2).cl_file = 'CS2_cl.txt'; rotor_template.c81_pairs(2).cd_file = 'CS2_cd.txt';
        rotor_template.c81_pairs(3).cl_file = 'CS3_cl.txt'; rotor_template.c81_pairs(3).cd_file = 'CS3_cd.txt';
        rotor_template.c81_pairs(4).cl_file = 'CS4_cl.txt'; rotor_template.c81_pairs(4).cd_file = 'CS4_cd.txt';
        rotor_template.c81_pairs(5).cl_file = 'CS5_cl.txt'; rotor_template.c81_pairs(5).cd_file = 'CS5_cd.txt';
        rotor_template.c81_pairs(6).cl_file = 'CS6_cl.txt'; rotor_template.c81_pairs(6).cd_file = 'CS6_cd.txt';

    otherwise
        error('rotor_template.airfoil_model must be ''linear'' or ''c81txt''.');
end

switch lower(rotor_template.chord_model)
    case 'linear'
        rotor_template.chord_linear = struct( ...
            'type', 'linear', ...
            'unit', 'meter', ...
            'root_value', 0.18, ...
            'tip_value', 0.18);

    case 'lookup'
        rotor_template.chord_lookup = struct( ...
            'mat', 'chord_interp.mat', ...
            'var', 'F', ...
            'unit', 'c_over_R');

    otherwise
        error('rotor_template.chord_model must be ''linear'' or ''lookup''.');
end

switch lower(rotor_template.pretwist_model)
    case 'linear'
        rotor_template.pretwist_linear = struct( ...
            'type', 'linear', ...
            'root_deg', 0.0, ...
            'tip_deg', -12.0);

    case 'lookup'
        rotor_template.pretwist_lookup = struct( ...
            'mat', 'pretwist_interp.mat', ...
            'var', 'pre_twist');

    otherwise
        error('rotor_template.pretwist_model must be ''linear'' or ''lookup''.');
end

vehicle.rotors = repmat(rotor_template, 1, rotor_count);
for i = 1:rotor_count
    vehicle.rotors(i).hub_loc = rotor_positions_m(i, :).';
    vehicle.rotors(i).rotational_direction = rotational_direction(i);
end

%% Airframe switch.
%   'lookup_combined' : legacy style. Whole fuselage+wing as one lookup body,
%                       plus elevator/rudder/aileron increment tables.
%   'component_bem'   : configurable wing/tail lifting surfaces plus a
%                       drag-only fuselage component.
airframe = struct();
if use_all_default_models
    airframe.model = 'component_bem';
else
    airframe.model = 'lookup_combined';
end

switch lower(airframe.model)
    case 'lookup_combined'
        airframe.lookup = struct();
        airframe.lookup.reference_m = [0; 0; 0];
        airframe.lookup.tilt_angle_deg = rotor_template.tilt_angle_deg;
        airframe.lookup.area_m2 = 12.95;
        airframe.lookup.chord_m = 1.09;
        airframe.lookup.span_m = 12.0;
        airframe.lookup.files = struct( ...
            'cd', 'Fuselage_cd.txt', ...
            'cl', 'Fuselage_cl.txt', ...
            'cm', 'Fuselage_cm.txt', ...
            'cc', 'Fuselage_cc.txt', ...
            'cn', 'Fuselage_cn.txt', ...
            'cll', 'Fuselage_cll.txt');
        airframe.lookup.control_files = struct( ...
            'elevator', 'Fuselage_elevator.txt', ...
            'rudder', 'Fuselage_rudder.txt', ...
            'aileron', 'Fuselage_roll.txt');

    case 'component_bem'
        airframe.components = default_airframe_components();

    otherwise
        error('airframe.model must be ''lookup_combined'' or ''component_bem''.');
end
vehicle.airframe = airframe;

%% Trim/stability settings.
% This is the place to edit trim sweep and initial values.
options = struct();
options.max_iter = 40;
options.tol = 1e-10;
options.fd_step = 1e-4;
options.damping = 1.0;
options.min_damping = 0.15;

options.trim = struct();
options.trim.tilt_angle_deg = tilt_angle_deg;
options.trim.n_speed_points = 6;
options.trim.speed_start_mps = 40;
options.trim.speed_step_mps = 10;
options.trim.trim_max_iter = 10000;
options.trim.stability_max_iter = 10000;
options.trim.keep_generated_script = false;

options.trim.uvw_earth_mps = state.velo_body;
options.trim.pqr_rad_s = state.angular_velocity_body;
options.trim.acceleration_body = state.acceleration_body;
options.trim.angular_acceleration_body = state.angular_acceleration_body;
options.trim.vehicle_control_rad = deg2rad([ ...
    state.control.elevator_deg, state.control.rudder_deg, state.control.aileron_deg]);

% Rotor trim initial value for each rotor:
%   beta_rad / beta_dot_rad: flapping states, length Nb or scalar
%   induced_velocity: old Trim_var(11), repeated for every rotor
% Fixed-control initial values:
%   collective/longitudinal/lateral/yaw are in degrees
%   pitch/roll are Euler angles in radians
options.trim.initial = struct();
options.trim.initial.beta_rad = zeros(rotor_template.Nb, 1);
options.trim.initial.beta_dot_rad = zeros(rotor_template.Nb, 1);
options.trim.initial.induced_velocity = 1.0;
options.trim.initial.collective_deg = rotor_template.theta0_deg;
options.trim.initial.longitudinal_deg = -0.002;
options.trim.initial.lateral_deg = 0.0;
options.trim.initial.yaw_deg = 0.0;
options.trim.initial.pitch_rad = -0.01;
options.trim.initial.roll_rad = 0.0;
%% ======================= END USER SETTINGS =======================

setenv("EVTOL_DATA_DIR", data_dir);

vehicle = evtol_configure_vehicle_models(vehicle, data_dir);

% Backward-compatible cfg/model/legacy are kept for data validation and the
% current legacy solver boundary.
cfg = make_compat_cfg(root, data_dir, state, vehicle, options);
model = evtol_load_model_data(cfg);
legacy = evtol_make_legacy_inputs(cfg, model);

if configure_only
    return;
end

fprintf("\n=== eVTOL BEMT package ===\n");
fprintf("Project root   : %s\n", root);
fprintf("Data folder    : %s\n", data_dir);
fprintf("Rotor count    : %d\n", numel(vehicle.rotors));
fprintf("Blade count    : %d\n", vehicle.rotors(1).Nb);
fprintf("Rotor airfoil  : %s\n", vehicle.rotors(1).airfoil_model);
fprintf("Rotor chord    : %s\n", vehicle.rotors(1).chord_model);
fprintf("Rotor pretwist : %s\n", vehicle.rotors(1).pretwist_model);
fprintf("Tilt angle     : %.3f deg\n", cfg.trim.tilt_angle_deg);
fprintf("Airframe model : %s\n\n", vehicle.airframe.model);

state_user = state;
vehicle_user = vehicle;
cfg_user = cfg;
model_user = model;
legacy_user = legacy;

lookup_data_required = cfg_uses_lookup_data(cfg);

if run_data_validation
    if ~lookup_data_required
        fprintf("Data validation skipped because all selected models use defaults.\n\n");
    else
    run(fullfile(root, "examples", "validate_data_formats.m"));
    fprintf("\n");
    end
end

if run_lookup_regression
    if is_default_lookup_regression_case(vehicle)
        run(fullfile(root, "examples", "compare_legacy_lookup_values.m"));
        run(fullfile(root, "examples", "compare_airframe_lookup_forces.m"));
    else
        fprintf("Lookup regression skipped because selected models/files differ from the legacy lookup setup.\n");
    end
    fprintf("\n");
end

state = state_user;
vehicle = vehicle_user;
cfg = cfg_user;
model = model_user;
legacy = legacy_user;

if run_trim_and_stability
    trim_entry = fullfile(root, "RUN_TRIM_AND_STABILITY.m");
    if exist(trim_entry, "file") ~= 2
        error("The legacy trim/stability solver is not included in this public no-data package.");
    end
    trim_results = RUN_TRIM_AND_STABILITY(cfg, options.trim);
else
    trim_results = [];
    fprintf("Full trim/stability sweep skipped. The public no-data package does not include the legacy trim solver.\n\n");
end

airframe_forces = evtol_airframe_forces(vehicle.airframe, state, state.control, vehicle.rho);

if show_config_preview
    print_config_preview(state, vehicle, legacy, airframe_forces);
end

assignin("base", "state", state);
assignin("base", "vehicle", vehicle);
assignin("base", "options", options);
assignin("base", "cfg", cfg);
assignin("base", "model", model);
assignin("base", "legacy", legacy);
assignin("base", "airframe_forces", airframe_forces);
assignin("base", "trim_results", trim_results);

fprintf("\nReady. Variables state, vehicle, options, cfg, model, legacy, airframe_forces, and trim_results were placed in the MATLAB base workspace.\n");
end

function components = default_airframe_components()
template = struct( ...
    'name', '', ...
    'type', 'lifting_surface', ...
    'surface_axis', 'horizontal', ...
    'area_m2', 1.0, ...
    'chord_m', 0.5, ...
    'span_m', 1.0, ...
    'loc_m', [0; 0; 0], ...
    'incidence_deg', 0.0, ...
    'a0_per_rad', 5.7, ...
    'alpha0_deg', 0.0, ...
    'cd0', 0.02, ...
    'drag_k', 0.02, ...
    'control', 'none', ...
    'control_sign', 1.0);

components = repmat(template, 1, 6);

components(1).name = 'wing_left';
components(1).surface_axis = 'horizontal';
components(1).area_m2 = 3.0;
components(1).chord_m = 0.75;
components(1).span_m = 4.0;
components(1).loc_m = [0.0; 2.5; 0.0];
components(1).control = 'aileron';
components(1).control_sign = 1.0;

components(2).name = 'wing_right';
components(2).surface_axis = 'horizontal';
components(2).area_m2 = 3.0;
components(2).chord_m = 0.75;
components(2).span_m = 4.0;
components(2).loc_m = [0.0; -2.5; 0.0];
components(2).control = 'aileron';
components(2).control_sign = -1.0;

components(3).name = 'horizontal_tail_left';
components(3).surface_axis = 'horizontal';
components(3).area_m2 = 1.0;
components(3).chord_m = 0.45;
components(3).span_m = 1.6;
components(3).loc_m = [3.5; 0.8; 0.2];
components(3).control = 'elevator';

components(4).name = 'horizontal_tail_right';
components(4).surface_axis = 'horizontal';
components(4).area_m2 = 1.0;
components(4).chord_m = 0.45;
components(4).span_m = 1.6;
components(4).loc_m = [3.5; -0.8; 0.2];
components(4).control = 'elevator';

components(5).name = 'vertical_tail';
components(5).surface_axis = 'vertical';
components(5).area_m2 = 1.1;
components(5).chord_m = 0.6;
components(5).span_m = 1.4;
components(5).loc_m = [3.4; 0.0; -0.2];
components(5).control = 'rudder';

components(6).name = 'fuselage';
components(6).type = 'fuselage_cd';
components(6).surface_axis = 'body';
components(6).area_m2 = 3.5;
components(6).chord_m = 1.0;
components(6).span_m = 1.0;
components(6).loc_m = [0.0; 0.0; 0.0];
components(6).cd0 = 0.12;
components(6).control = 'none';
end

function cfg = make_compat_cfg(root, data_dir, state, vehicle, options)
cfg = struct();

cfg.paths = struct();
cfg.paths.root = root;
cfg.paths.data_dir = data_dir;

cfg.environment = struct();
cfg.environment.rho = vehicle.rho;
cfg.environment.g = vehicle.g;

cfg.aircraft = struct();
cfg.aircraft.mass_kg = vehicle.mass_kg;
cfg.aircraft.inertia_kgm2 = vehicle.inertia_kgm2;
cfg.aircraft.cg_m = vehicle.cg_m(:).';
cfg.aircraft.fuselage_ref_m = [0, 0, 0];

cfg.rotors = struct();
cfg.rotors.count = numel(vehicle.rotors);
cfg.rotors.radius_m = vehicle.rotors(1).R;
cfg.rotors.blade_count = vehicle.rotors(1).Nb;
cfg.rotors.omega_rad_s = vehicle.rotors(1).omega;
cfg.rotors.I_beta = vehicle.rotors(1).I_beta;
cfg.rotors.k_beta = vehicle.rotors(1).k_beta;
cfg.rotors.root_cutout = vehicle.rotors(1).root_cutout;
cfg.rotors.n_blade_elements = vehicle.rotors(1).n_be;
cfg.rotors.n_azimuth_steps = vehicle.rotors(1).n_az;
cfg.rotors.position_mode = "manual";
cfg.rotors.positions_m = reshape([vehicle.rotors.hub_loc], 3, []).';
cfg.rotors.rotational_direction = [vehicle.rotors.rotational_direction];

cfg.data = struct();
cfg.data.airfoil = struct();
cfg.data.airfoil.section_r_end = vehicle.rotors(1).section_r_end;
cfg.data.airfoil.default = airfoil_default_from_rotor(vehicle.rotors(1));
switch lower(string(vehicle.rotors(1).airfoil_model))
    case "linear"
        cfg.data.airfoil.mode = "default";
    case "c81txt"
        cfg.data.airfoil.mode = "lookup";
        cfg.data.airfoil.cl_files = string({vehicle.rotors(1).c81_pairs.cl_file});
        cfg.data.airfoil.cd_files = string({vehicle.rotors(1).c81_pairs.cd_file});
    otherwise
        error("Unsupported rotor airfoil_model: %s", vehicle.rotors(1).airfoil_model);
end

cfg.data.chord = struct();
switch lower(string(vehicle.rotors(1).chord_model))
    case "linear"
        cfg.data.chord.mode = "default";
        cfg.data.chord.default_root_c_over_R = chord_to_c_over_R( ...
            vehicle.rotors(1).chord_linear.root_value, vehicle.rotors(1).chord_linear.unit, cfg.rotors.radius_m);
        cfg.data.chord.default_tip_c_over_R = chord_to_c_over_R( ...
            vehicle.rotors(1).chord_linear.tip_value, vehicle.rotors(1).chord_linear.unit, cfg.rotors.radius_m);
    case "lookup"
        cfg.data.chord.mode = "lookup";
        cfg.data.chord.lookup_file = string(vehicle.rotors(1).chord_lookup.mat);
        cfg.data.chord.lookup_var = string(vehicle.rotors(1).chord_lookup.var);
        cfg.data.chord.default_root_c_over_R = 0.18 / cfg.rotors.radius_m;
        cfg.data.chord.default_tip_c_over_R = 0.18 / cfg.rotors.radius_m;
    otherwise
        error("Unsupported rotor chord_model: %s", vehicle.rotors(1).chord_model);
end

cfg.data.pretwist = struct();
switch lower(string(vehicle.rotors(1).pretwist_model))
    case "linear"
        cfg.data.pretwist.mode = "default";
        cfg.data.pretwist.default_root_deg = vehicle.rotors(1).pretwist_linear.root_deg;
        cfg.data.pretwist.default_tip_deg = vehicle.rotors(1).pretwist_linear.tip_deg;
    case "lookup"
        cfg.data.pretwist.mode = "lookup";
        cfg.data.pretwist.lookup_file = string(vehicle.rotors(1).pretwist_lookup.mat);
        cfg.data.pretwist.lookup_var = string(vehicle.rotors(1).pretwist_lookup.var);
        cfg.data.pretwist.default_root_deg = 0.0;
        cfg.data.pretwist.default_tip_deg = -12.0;
    otherwise
        error("Unsupported rotor pretwist_model: %s", vehicle.rotors(1).pretwist_model);
end

cfg.data.fuselage = struct();
cfg.data.controls = struct();
cfg.data.fuselage.default = struct("cd0", 0.08, "drag_k", 0.8, ...
    "cl_alpha_per_rad", 2.5, "cm_alpha_per_rad", -0.15, ...
    "cy_beta_per_rad", -0.7, "cn_beta_per_rad", 0.08, ...
    "cll_beta_per_rad", -0.08);
cfg.data.controls.default = struct("elevator_dcl_per_rad", 0.4, ...
    "elevator_dcm_per_rad", -0.7, "rudder_dcy_per_rad", 0.25, ...
    "rudder_dcn_per_rad", -0.08, "aileron_dcll_per_rad", 0.12);

if strcmpi(vehicle.airframe.model, "lookup_combined")
    cfg.data.fuselage.mode = "lookup";
    cfg.data.controls.mode = "lookup";
    cfg.data.fuselage.area_m2 = vehicle.airframe.lookup.area_m2;
    cfg.data.fuselage.chord_m = vehicle.airframe.lookup.chord_m;
    cfg.data.fuselage.span_m = vehicle.airframe.lookup.span_m;
    cfg.data.fuselage.files = vehicle.airframe.lookup.files;
    cfg.data.controls.files = vehicle.airframe.lookup.control_files;
else
    cfg.data.fuselage.mode = "default";
    cfg.data.controls.mode = "default";
    cfg.data.fuselage.area_m2 = 12.95;
    cfg.data.fuselage.chord_m = 1.09;
    cfg.data.fuselage.span_m = 12.0;
    cfg.data.fuselage.files = struct("cd", "Fuselage_cd.txt", "cl", "Fuselage_cl.txt", ...
        "cm", "Fuselage_cm.txt", "cc", "Fuselage_cc.txt", "cn", "Fuselage_cn.txt", ...
        "cll", "Fuselage_cll.txt");
    cfg.data.controls.files = struct("elevator", "Fuselage_elevator.txt", ...
        "rudder", "Fuselage_rudder.txt", "aileron", "Fuselage_roll.txt");
end

cfg.trim = options.trim;
cfg.trim.uvw_earth_mps = cfg.trim.uvw_earth_mps(:);
cfg.trim.pqr_rad_s = cfg.trim.pqr_rad_s(:);
cfg.trim.acceleration_body = cfg.trim.acceleration_body(:);
cfg.trim.angular_acceleration_body = cfg.trim.angular_acceleration_body(:);
cfg.trim.vehicle_control_rad = cfg.trim.vehicle_control_rad(:).';
cfg.trim.initial_vector = build_trim_initial_vector(cfg.rotors.count, cfg.rotors.blade_count, cfg.trim.initial);

if isempty(state)
    error("Invalid state input.");
end
end

function d = airfoil_default_from_rotor(rotor)
if isfield(rotor, "airfoil_linear")
    d = struct("a0_per_rad", rotor.airfoil_linear.a0, ...
        "alpha0_deg", rotor.airfoil_linear.alpha0_deg, ...
        "cd0", rotor.airfoil_linear.cd0, ...
        "drag_k", rotor.airfoil_linear.k);
else
    d = struct("a0_per_rad", 5.7, "alpha0_deg", 0.0, ...
        "cd0", 0.010, "drag_k", 0.020);
end
end

function value = chord_to_c_over_R(value, unit, radius_m)
if strcmpi(unit, "meter")
    value = value / radius_m;
end
end

function trim_vector = build_trim_initial_vector(rotor_count, blade_count, initial)
rotor_state_len = 2 * blade_count + 1;
trim_vector = zeros(rotor_count * rotor_state_len + 6, 1);

beta = expand_to_length(initial.beta_rad, blade_count, "beta_rad");
beta_dot = expand_to_length(initial.beta_dot_rad, blade_count, "beta_dot_rad");
rotor_state = [beta; beta_dot; initial.induced_velocity];

for i = 1:rotor_count
    idx = (i - 1) * rotor_state_len + (1:rotor_state_len);
    trim_vector(idx) = rotor_state;
end

base = rotor_count * rotor_state_len;
trim_vector(base + (1:6)) = [ ...
    initial.collective_deg; ...
    initial.longitudinal_deg; ...
    initial.lateral_deg; ...
    initial.yaw_deg; ...
    initial.pitch_rad; ...
    initial.roll_rad];
end

function x = expand_to_length(x, n, label)
x = x(:);
if isscalar(x)
    x = repmat(x, n, 1);
end
if numel(x) ~= n
    error("options.trim.initial.%s must be scalar or length %d.", label, n);
end
end

function tf = is_default_lookup_regression_case(vehicle)
r = vehicle.rotors(1);
tf = strcmpi(r.airfoil_model, "c81txt") && ...
     strcmpi(r.chord_model, "lookup") && ...
     strcmpi(r.chord_lookup.mat, "chord_interp.mat") && ...
     strcmpi(r.chord_lookup.var, "F") && ...
     strcmpi(r.pretwist_model, "lookup") && ...
     strcmpi(r.pretwist_lookup.mat, "pretwist_interp.mat") && ...
     strcmpi(r.pretwist_lookup.var, "pre_twist") && ...
     strcmpi(vehicle.airframe.model, "lookup_combined");
end

function tf = cfg_uses_lookup_data(cfg)
tf = strcmpi(cfg.data.airfoil.mode, "lookup") || ...
     strcmpi(cfg.data.chord.mode, "lookup") || ...
     strcmpi(cfg.data.pretwist.mode, "lookup") || ...
     strcmpi(cfg.data.fuselage.mode, "lookup") || ...
     strcmpi(cfg.data.controls.mode, "lookup");
end

function print_config_preview(state, vehicle, legacy, airframe_forces)
fprintf("Configuration preview\n");
disp("Rotor hub positions [m]:");
disp(reshape([vehicle.rotors.hub_loc], 3, []).');
fprintf("Rotor count: %d\n", numel(vehicle.rotors));
fprintf("Blade count per rotor: %d\n", vehicle.rotors(1).Nb);
fprintf("Chord c/R at x=0.5: %.6f\n", legacy.F(0.5));
fprintf("Pretwist at x=0.5 [deg]: %.6f\n", legacy.pre_twist(0.5));
fprintf("CL at alpha=5 deg, Mach=0.2: %.6f\n", legacy.cl{1}(5, 0.2));
fprintf("CD at alpha=5 deg, Mach=0.2: %.6f\n", legacy.cd{1}(5, 0.2));
fprintf("State velo_body [m/s]: [%.3f %.3f %.3f]\n", state.velo_body);
fprintf("Airframe forces [Fx Fy Fz Mx My Mz]^T:\n");
disp(airframe_forces);
end
