if exist('flap_model_override', 'var')
    requested_flap_model = string(flap_model_override);
else
    requested_flap_model = "disk";
end
if exist('response_dt_s_override', 'var')
    requested_response_dt_s = response_dt_s_override;
else
    requested_response_dt_s = response_default_dt_s();
end
clearvars -except requested_flap_model requested_response_dt_s;
clc;

root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'src'));
data_dir = fullfile(root, 'data');

% -------------------------------------------------------------------------
% Simulink response setup
% -------------------------------------------------------------------------
% This script trims the aircraft and exports initial conditions plus a
% simulation model struct for VTOL_RESPONSE_STEP.m.

data_mode = "default";          % "default" or "lookup" for geometry/rotor data
aero_database_mode = data_mode; % "default", "lookup", or "excel" for fuselage/control aero

cfg = struct();

cfg.switch.geometry = data_mode;
cfg.switch.rotor_positions = data_mode;
cfg.switch.chord = data_mode;
cfg.switch.pretwist = data_mode;
cfg.switch.airfoil = data_mode;
cfg.switch.fuselage = aero_database_mode;
cfg.switch.controls = aero_database_mode;
cfg.data.geometry.cg_file = 'CG_positions.txt';
cfg.data.geometry.rotor_positions_file = 'Rotor_positions.txt';
cfg.data.fuselage.reference_point_mm = [3600 0 0];
cfg.data.aero.excel_file = 'V16_aero_database_clean.xlsx';
cfg.data.aero.base_sheet = 'base_aero';
cfg.data.aero.control_surface_sheets = {'WL1','WL2','WR1','WR2','VL1','VL2','VR1','VR2'};
cfg.data.chord.txt_file = 'Chord.txt';
cfg.data.chord.mat_var = 'F';
cfg.data.pretwist.txt_file = 'Pretwist.txt';
cfg.data.pretwist.mat_var = 'pre_twist';

cfg.environment.rho_kg_m3 = 1.225;
cfg.environment.gravity_m_s2 = 9.81;
cfg.vehicle.mass_kg = 1900;
cfg.vehicle.inertia_kg_m2 = [1966.5, 5245.3, 3282.7];

cfg.rotor.radius_m = 1.5;
cfg.rotor.blade_count = 5;
cfg.rotor.omega_rad_s = 90;
cfg.rotor.flap_inertia_kg_m2 = 2.25;
cfg.rotor.flap_spring_nm_rad = 16000;
cfg.rotor.airfoil_section_edges = [0.25 0.40 0.50 0.80 0.92];
cfg.rotor.rotational_direction = [1 -1 1 -1 1 -1];
cfg.rotor.blade_element_count = 10;
cfg.rotor.azimuth_steps = 72;
cfg.rotor.flap_integrator = "euler";
cfg.rotor.flap_model = requested_flap_model;  % "blade" or "disk"
cfg.rotor.inflow_model = "uniform";
cfg.rotor.inflow_tau_mode = "pitt";
cfg.rotor.inflow_tau_s = 0.02;
cfg.rotor.inflow_min_lambda = 0.02;

cfg.fuselage.reference_area_m2 = 14.41;
cfg.fuselage.mean_aero_chord_m = 1.31;
cfg.fuselage.span_m = 12;
cfg.controls.channel_to_physical_gain = 0.5;
cfg.controls.channel_names = {'pitch','yaw','roll'};
cfg.controls.physical_surface_names = {'WL1','WL2','WR1','WR2','VL1','VL2','VR1','VR2'};
cfg.controls.surface_mixing_matrix = [];
cfg.controls.surface_bias_deg = [];

cfg.aero.dynamic_derivatives.enabled = true;
cfg.aero.dynamic_derivatives.CLq = 7.3939521;
cfg.aero.dynamic_derivatives.Cmq = -16.8207899;
cfg.aero.dynamic_derivatives.Clp = -0.6099156;
cfg.aero.dynamic_derivatives.Cnp = 0.0251017;
cfg.aero.dynamic_derivatives.Cyp = -0.1026634;
cfg.aero.dynamic_derivatives.Cnr = -0.1027184;
cfg.aero.dynamic_derivatives.Clr = 0.0608574;
cfg.aero.dynamic_derivatives.Cyr = 0.3244426;
cfg.aero.dynamic_derivatives.Cma_dot = -8.41039495;
cfg.aero.dynamic_derivatives.Cn_beta_dot = -0.0513592;
cfg.aero.dynamic_derivatives.alpha_beta_dot_mode = "kinematic";
cfg.aero.dynamic_derivatives.alpha_beta_dot_mode_id = 1;
cfg.aero.dynamic_derivatives.min_velocity_mps = 1e-6;

cfg.trim.tilt_angle_deg = 90;
cfg.rotor.tilt_angle_deg = cfg.trim.tilt_angle_deg;
cfg.trim.speed_mps = 0;
cfg.trim.use_previous_solution = true;
cfg.trim.max_iterations = 30;
cfg.trim.tol = 1e-7;
cfg.stability.max_iterations = 1;
cfg.stability.tol = 1e-7;
cfg.output.verbose = false;

cfg.initial.uvw_earth_mps = [cfg.trim.speed_mps(1) 0 0];
cfg.initial.pqr_rad_s = [0 0 0];
cfg.initial.fixed_wing_control = [0 0 0];

cfg.trim.initial.rotor_state = [zeros(2*cfg.rotor.blade_count, 1); 5];
cfg.trim.initial.collective_deg = 18;
cfg.trim.initial.longitudinal_deg = -0.002;
cfg.trim.initial.lateral_deg = 0;
cfg.trim.initial.yaw_deg = 0;
cfg.trim.initial.pitch_rad = -0.01;
cfg.trim.initial.roll_rad = 0;

cfg.response.enabled = true;
cfg.response.skip_stability = true;
cfg.response.duration_s = 0;
cfg.response.dt_s = requested_response_dt_s;
cfg.response.aircraft_integrator = "rk4";
cfg.response.update_rotor_states = true;
cfg.response.initial_state_delta = zeros(12,1);
cfg.response.control_delta = zeros(4,1);
cfg.response.rotor_tilt_angle_deg = [];
cfg.response.fixed_wing_control_delta = zeros(3,1); % fixed-wing [pitch yaw roll] channel increments, deg
cfg.response.step_time_s = 0;
cfg.response.compile_mex = lower(string(cfg.rotor.flap_model)) == "disk";

run(fullfile(root, 'src', 'BEMTFLAP_SWITCHED.m'));

sim_init = build_simulink_response_init(trim_results);

assign_response_base(sim_init);

if cfg.response.compile_mex
    sim_init = compile_fast_disk_mex_for_current_setup(root, sim_init);
    assign_response_base(sim_init);
    CREATE_RESPONSE_SIMULINK_MEX_MODEL;
end

fprintf('\n=== Simulink response setup complete ===\n');
fprintf('trim status: %s\n', trim_results.status);
fprintf('trim residual norm: %.12g\n', trim_results.trim_residual_norm);
fprintf('x0 size: %dx%d\n', size(sim_init.x0,1), size(sim_init.x0,2));
fprintf('rotor_state0 size: %dx%d\n', size(sim_init.rotor_state0,1), size(sim_init.rotor_state0,2));
fprintf('base workspace variables updated: sim_init, x0, rotor_state0, dt_sim, sim_model\n');
if cfg.response.compile_mex
    fprintf('MEX rebuilt for current setup: %s\n', fullfile(root, 'VTOL_RESPONSE_SIM_STEP_disk_fast_mex.mexw64'));
end

function assign_response_base(sim_init)
assignin('base', 'sim_init', sim_init);
assignin('base', 'x0', sim_init.x0);
assignin('base', 'rotor_state0', sim_init.rotor_state0);
assignin('base', 'control_trim', sim_init.control_trim_vector);
assignin('base', 'control_delta', sim_init.control_delta(:));
assignin('base', 'rotor_tilt_angle_deg', sim_init.rotor_tilt_angle_deg(:));
assignin('base', 'fixed_control_delta', sim_init.fixed_control_delta(:));
assignin('base', 'dt_sim', sim_init.dt_s);
assignin('base', 'sim_model', sim_init.model);
end

function sim_init = compile_fast_disk_mex_for_current_setup(root, sim_init)
sim_init.control_delta = zeros(4,1);
sim_init.fixed_control_delta = zeros(3,1);
sim_init.model.disk_state_substeps = 8;
sim_init.model = make_mex_compatible_sim_model(sim_init.model, false);

assign_response_base(sim_init);
clear VTOL_RESPONSE_SIM_STEP_disk_fast_mex

cfg_mex = coder.config('mex');
cfg_mex.GenerateReport = false;

args = {0, ...
    sim_init.x0, ...
    sim_init.rotor_state0, ...
    sim_init.control_delta(:), ...
    sim_init.rotor_tilt_angle_deg(:), ...
    sim_init.fixed_control_delta(:), ...
    sim_init.dt_s, ...
    coder.Constant(sim_init.model)};

fprintf('\n=== Rebuilding fast disk MEX for current setup ===\n');
codegen('-config', cfg_mex, ...
    'VTOL_RESPONSE_SIM_STEP', ...
    '-args', args, ...
    '-o', 'VTOL_RESPONSE_SIM_STEP_disk_fast_mex', ...
    '-d', fullfile(root, 'codegen_mex_response_disk_fast'));
end

function sim_init = build_simulink_response_init(trim_results)
cfg = trim_results.cfg;
sim_model = trim_results.response.sim_model;
Nb = cfg.rotor.blade_count;
n_rotors = 6;
trim_rotor_state_size = (numel(trim_results.final_trim_var) - 6) / n_rotors;
control_start = n_rotors * trim_rotor_state_size + 1;

trim_var = trim_results.final_trim_var;
x0 = trim_results.response.state_history(1,:).';
control_trim_rotor = trim_var(control_start:control_start+3);
control_trim_fixed = cfg.initial.fixed_wing_control(:);

flap_model = lower(string(cfg.rotor.flap_model));
switch flap_model
    case "blade"
        response_rotor_state_size = 2*Nb + 1;
        rotor_state0 = trim_var(1:n_rotors*trim_rotor_state_size);
        force_fun = @bemt_flapp;
    case "disk"
        response_rotor_state_size = 7;
        rotor_state0 = zeros(n_rotors * response_rotor_state_size, 1);
        for rotor_number = 1:n_rotors
            trim_idx = (rotor_number-1)*trim_rotor_state_size + (1:trim_rotor_state_size);
            response_idx = (rotor_number-1)*response_rotor_state_size + (1:response_rotor_state_size);
            if trim_rotor_state_size == response_rotor_state_size
                rotor_state0(response_idx) = trim_var(trim_idx);
            else
                rotor_state0(response_idx) = blade_to_disk_flap_state(trim_var(trim_idx), ...
                    Nb, cfg.rotor.omega_rad_s, sim_model.rotational_direction(rotor_number));
            end
        end
        force_fun = @bemt_flapp_disk;
    otherwise
        error('RUN_RESPONSE_SIMULINK_SETUP:BadFlapModel', ...
            'cfg.rotor.flap_model must be "blade" or "disk".');
end

sim_model.control_trim.rotor = control_trim_rotor(:);
sim_model.control_trim.fixed = control_trim_fixed(:);
sim_model.aircraft_integrator = cfg.response.aircraft_integrator;
sim_model.flap_model = flap_model;
sim_model.flap_model_id = response_flap_model_id(flap_model);
sim_model.disk_flap_state_update = "dynamic";
sim_model.disk_flap_state_update_id = 1;
sim_model.disk_state_integrator = "rk4";
sim_model.disk_state_substeps = 2;
if isfield(cfg.response, 'disk_flap_state_update')
    sim_model.disk_flap_state_update = lower(string(cfg.response.disk_flap_state_update));
    sim_model.disk_flap_state_update_id = response_disk_update_id(sim_model.disk_flap_state_update);
end
if isfield(cfg.response, 'disk_state_integrator')
    sim_model.disk_state_integrator = lower(string(cfg.response.disk_state_integrator));
end
sim_model.disk_state_integrator_id = response_rotor_integrator_id(sim_model.disk_state_integrator);
if isfield(cfg.response, 'disk_state_substeps')
    sim_model.disk_state_substeps = cfg.response.disk_state_substeps;
end
sim_model.rotor_state_size = response_rotor_state_size;
sim_model.force_fun = force_fun;
sim_model.fuselage_fun = @fuselage_aerodynamics;
sim_model.rotor_bemt_options.flap_integrator_id = response_rotor_integrator_id(sim_model.rotor_bemt_options.flap_integrator);
sim_model.rotor_bemt_options.inflow_model_id = response_inflow_model_id(sim_model.rotor_bemt_options.inflow_model);
sim_model.rotor_bemt_options.inflow_tau_mode_id = response_inflow_tau_mode_id(sim_model.rotor_bemt_options.inflow_tau_mode);
sim_model.default_fuselage = cfg.defaults.fuselage;
sim_model.default_controls = cfg.defaults.controls;

sim_init = struct();
sim_init.x0 = x0;
sim_init.rotor_state0 = rotor_state0;
sim_init.control_trim_vector = [control_trim_rotor(:); control_trim_fixed(:)];
sim_init.control_delta = cfg.response.control_delta(:);
sim_init.rotor_tilt_angle_deg = response_rotor_tilt_angles(cfg, trim_results.rotor_tilt_angle_deg(:));
sim_init.fixed_control_delta = cfg.response.fixed_wing_control_delta(:);
sim_init.dt_s = cfg.response.dt_s;
sim_init.model = sim_model;
sim_init.state_order = trim_results.response.state_order;
sim_init.control_delta_order = trim_results.response.control_order;
sim_init.trim_results = trim_results;
end

function model_id = response_flap_model_id(flap_model)
switch lower(string(flap_model))
    case "blade"
        model_id = 1;
    case "disk"
        model_id = 2;
    otherwise
        error('RUN_RESPONSE_SIMULINK_SETUP:BadFlapModel', ...
            'cfg.rotor.flap_model must be "blade" or "disk".');
end
end

function update_id = response_disk_update_id(disk_update)
switch lower(string(disk_update))
    case "dynamic"
        update_id = 1;
    case "frozen"
        update_id = 2;
    otherwise
        error('RUN_RESPONSE_SIMULINK_SETUP:BadDiskUpdate', ...
            'cfg.response.disk_flap_state_update must be "dynamic" or "frozen".');
end
end

function integrator_id = response_rotor_integrator_id(integrator_name)
switch lower(string(integrator_name))
    case "euler"
        integrator_id = 1;
    case "rk4"
        integrator_id = 2;
    otherwise
        error('RUN_RESPONSE_SIMULINK_SETUP:BadRotorIntegrator', ...
            'Rotor integrator must be "euler" or "rk4".');
end
end

function model_id = response_inflow_model_id(inflow_model)
switch lower(string(inflow_model))
    case "uniform"
        model_id = 1;
    otherwise
        error('RUN_RESPONSE_SIMULINK_SETUP:BadInflowModel', ...
            'cfg.rotor.inflow_model must be "uniform".');
end
end

function mode_id = response_inflow_tau_mode_id(inflow_tau_mode)
switch lower(string(inflow_tau_mode))
    case "manual"
        mode_id = 1;
    case "pitt"
        mode_id = 2;
    otherwise
        error('RUN_RESPONSE_SIMULINK_SETUP:BadInflowTau', ...
            'cfg.rotor.inflow_tau_mode must be "manual" or "pitt".');
end
end

function rotor_tilt_angle_deg = response_rotor_tilt_angles(cfg, trim_rotor_tilt_angle_deg)
if isfield(cfg.response, 'rotor_tilt_angle_deg') && ~isempty(cfg.response.rotor_tilt_angle_deg)
    rotor_tilt_angle_deg = cfg.response.rotor_tilt_angle_deg(:);
else
    rotor_tilt_angle_deg = trim_rotor_tilt_angle_deg(:);
end
if numel(rotor_tilt_angle_deg) ~= 6 || any(~isfinite(rotor_tilt_angle_deg))
    error('RUN_RESPONSE_SIMULINK_SETUP:BadRotorTiltAngle', ...
        'cfg.response.rotor_tilt_angle_deg must be empty or contain six finite values in degrees.');
end
end
