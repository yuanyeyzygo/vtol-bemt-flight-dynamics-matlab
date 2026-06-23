clear;
clc;

root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'src'));
data_dir = fullfile(root, 'data');

% -------------------------------------------------------------------------
% RUN_ME interface
% -------------------------------------------------------------------------
% This package now uses the original BEMTFLAP calculation flow. The switches
% below only choose lookup data or built-in default values.
%
% Use "lookup" when the private data files are available in ./data.
% Use "default" when publishing/running without private data files.

data_mode = "default";          % "default" or "lookup" for geometry/rotor data
aero_database_mode = data_mode; % "default", "lookup", or "excel" for fuselage/control aero

cfg = struct();

% Data source switches
cfg.switch.geometry = data_mode;   % CG_positions.txt or defaults
cfg.switch.rotor_positions = data_mode; % Rotor_positions.txt or analytic defaults
cfg.switch.chord = data_mode;      % Chord.txt or constant chord
cfg.switch.pretwist = data_mode;   % Pretwist.txt or linear twist
cfg.switch.airfoil = data_mode;    % CS*_cl/cd.txt or default CL/CD formulas
cfg.switch.fuselage = aero_database_mode; % fuselage coefficient tables, Excel, or defaults
cfg.switch.controls = aero_database_mode; % elevator/rudder/aileron or WL/WR/VL/VR tables
cfg.data.geometry.cg_file = 'CG_positions.txt';
cfg.data.geometry.rotor_positions_file = 'Rotor_positions.txt';
cfg.data.fuselage.reference_point_mm = [3600 0 0];  % V11 aero moment reference point, mm
cfg.data.aero.excel_file = 'V16_aero_database_clean.xlsx';
cfg.data.aero.base_sheet = 'base_aero';
cfg.data.aero.control_surface_sheets = {'WL1','WL2','WR1','WR2','VL1','VL2','VR1','VR2'};
cfg.data.chord.txt_file = 'Chord.txt';              % r/R, chord_m
%cfg.data.chord.mat_file = 'chord_interp.mat';       % legacy c/R fallback
cfg.data.chord.mat_var = 'F';
cfg.data.pretwist.txt_file = 'Pretwist.txt';        % r/R, twist_deg
%cfg.data.pretwist.mat_file = 'pretwist_interp.mat';
cfg.data.pretwist.mat_var = 'pre_twist';

% Environment and vehicle
cfg.environment.rho_kg_m3 = 1.225;
cfg.environment.gravity_m_s2 = 9.81;
cfg.vehicle.mass_kg = 1900;
cfg.vehicle.inertia_kg_m2 = [1966.5, 5245.3, 3282.7]; % [Ixx Iyy Izz]

% Rotor settings
cfg.rotor.radius_m = 1.5;
cfg.rotor.blade_count = 5;
cfg.rotor.omega_rad_s = 90;
cfg.rotor.flap_inertia_kg_m2 = 2.25;
cfg.rotor.flap_spring_nm_rad = 16000;
cfg.rotor.airfoil_section_edges = [0.25 0.40 0.50 0.80 0.92];
cfg.rotor.rotational_direction = [1 -1 1 -1 1 -1]; % +1 or -1 for rotors 1..6
cfg.rotor.blade_element_count = 10;
cfg.rotor.azimuth_steps = 72;
cfg.rotor.flap_integrator = "euler"; % "euler" preserves legacy trim/stability results; use "rk4" for response runs
cfg.rotor.flap_model = "blade";      % "blade" or "disk"
cfg.rotor.inflow_model = "fixed";    % "fixed" preserves legacy response; "uniform" adds first-order dynamic inflow
cfg.rotor.inflow_tau_mode = "pitt";  % "pitt" or "manual"
cfg.rotor.inflow_tau_s = 0.02;       % used when inflow_tau_mode = "manual"
cfg.rotor.inflow_min_lambda = 0.02;  % lower bound for Pitt tau calculation

% Fuselage/aero database reference geometry
cfg.fuselage.reference_area_m2 = 14.41;
cfg.fuselage.mean_aero_chord_m = 1.31;
cfg.fuselage.span_m = 12;
cfg.controls.channel_to_physical_gain = 0.5; % channel -> VL/VR pitch/yaw gain; set 1.0 if no /2 is desired
cfg.controls.channel_names = {'pitch','yaw','roll'};
cfg.controls.physical_surface_names = {'WL1','WL2','WR1','WR2','VL1','VL2','VR1','VR2'};
% Rows are physical surfaces above; columns are channel_names above.
% Leave empty to use the default mapping from the report:
%   VL = 0.5*(yaw - pitch), VR = 0.5*(yaw + pitch), WL/WR = roll.
cfg.controls.surface_mixing_matrix = [];
cfg.controls.surface_bias_deg = [];

% Fuselage/wing dynamic aerodynamic derivatives. Rates use the standard
% nondimensional forms p*b/(2V), q*c/(2V), r*b/(2V).
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
cfg.aero.dynamic_derivatives.alpha_beta_dot_mode = "kinematic"; % "kinematic" or "zero"
cfg.aero.dynamic_derivatives.alpha_beta_dot_mode_id = 1;        % 1=kinematic, 0=zero
cfg.aero.dynamic_derivatives.min_velocity_mps = 1e-6;

% Nonlinear response settings. RUN_ME leaves this disabled; RUN_RESPONSE.m
% enables it and skips the stability derivative calculation.
cfg.response.enabled = false;
cfg.response.skip_stability = false;
cfg.response.duration_s = 5;
cfg.response.dt_s = 0.05;
cfg.response.aircraft_integrator = "rk4";
cfg.response.update_rotor_states = true;
cfg.response.initial_state_delta = zeros(12,1); % [u v w p q r phi theta psi x y z]
cfg.response.control_delta = zeros(4,1);        % [collective longitudinal lateral yaw], deg
cfg.response.rotor_tilt_angle_deg = [];         % empty -> use trim rotor tilt angles, deg
cfg.response.fixed_wing_control_delta = zeros(3,1); % fixed-wing [pitch yaw roll] channel increments, deg
cfg.response.step_time_s = 0;

% Trim case
cfg.trim.tilt_angle_deg = 90;       % scalar airframe/CG/fuselage lookup tilt
cfg.rotor.tilt_angle_deg = cfg.trim.tilt_angle_deg; % scalar or 1x6 per-rotor nacelle tilts
% Example for individual nacelles:
% cfg.rotor.tilt_angle_deg = [90 90 90 90 85 85];
cfg.trim.speed_mps = [0 10 20];      % forward speeds to calculate
cfg.trim.use_previous_solution = true; % warm-start each speed from the previous trim result
cfg.trim.max_iterations = 10000;
cfg.stability.max_iterations = 10000;
cfg.output.verbose = false;

% Initial state and controls
cfg.initial.uvw_earth_mps = [cfg.trim.speed_mps(1) 0 0];
cfg.initial.pqr_rad_s = [0 0 0];
cfg.initial.fixed_wing_control = [0 0 0]; % fixed-wing [pitch yaw roll] channels, deg

% Trim initial guesses. The 11-state rotor vector is repeated for all 6 rotors:
% [beta0(1:Nb) dbeta0(1:Nb) induced_velocity]
cfg.trim.initial.rotor_state = [zeros(2*cfg.rotor.blade_count, 1); 5];
cfg.trim.initial.collective_deg = 18;
cfg.trim.initial.longitudinal_deg = -0.002;
cfg.trim.initial.lateral_deg = 0;
cfg.trim.initial.yaw_deg = 0;
cfg.trim.initial.pitch_rad = -0.01;
cfg.trim.initial.roll_rad = 0;

% Default data used when a switch is "default"
cfg.defaults.geometry.x_cg_mm = 3554.3;
cfg.defaults.geometry.y_cg_mm = 0;
cfg.defaults.geometry.z_cg_mm = -588.7;
cfg.defaults.geometry.first_rotor_x_mm = 430;
cfg.defaults.geometry.first_rotor_z_mm = -1900;
% Default rotor-position formula coefficients, one row per rotor:
% [x0 x_cos x_sin y z0 z_cos z_sin], mm, where
% x = x0 + x_cos*cos(tilt) + x_sin*sin(tilt)
% z = z0 + z_cos*cos(tilt) + z_sin*sin(tilt)
cfg.defaults.geometry.rotor_position_coeffs_mm = [
    3600.0  -750.0      0.0   6000  -1290.0     0.0  -750.0
    1303.4  -269.3    -47.5   2500  -1303.4   -40.7  -542.5
    1303.4  -269.3    -47.5  -2500  -1303.4   -40.7  -542.5
    3600.0  -750.0      0.0  -6000  -1290.0     0.0  -750.0
    5896.6   269.3     47.5   2500  -1396.6    40.7   542.5
    5896.6   269.3     47.5  -2500  -1396.6    40.7   542.5];

cfg.defaults.chord_m = 0.20264354;
cfg.defaults.pretwist_root_deg = 17.45;
cfg.defaults.pretwist_tip_deg = -4.13;

cfg.defaults.airfoil.cl_alpha_per_rad = 5.579842;
cfg.defaults.airfoil.cl_max = 1.134702;
cfg.defaults.airfoil.cd0 = 0.010150;
cfg.defaults.airfoil.cd_alpha2 = 1.758467;

cfg.defaults.fuselage.cd0 = 0.1417935;
cfg.defaults.fuselage.cl_alpha_per_rad = 7.1276989;
cfg.defaults.fuselage.cm_alpha_per_rad = 0.1212486;
cfg.defaults.fuselage.cc_beta = -0.1012928;
cfg.defaults.fuselage.cn_beta = 0.00368368;
cfg.defaults.fuselage.cll_beta = -0.0155062;

cfg.defaults.controls.elevator = [0.00458257, 0.11334677, -0.33342761]; % [dCD dCL dCM] per rad
cfg.defaults.controls.rudder = [-0.09308717, 0.03302849, -0.02026052];  % [dCC dCN dCLL] per rad
cfg.defaults.controls.aileron = [0.01919035, 0.00835123, -0.21814321];  % [dCC dCN dCLL] per rad

fprintf('Requested trim speeds [m/s]:');
fprintf(' %.6g', cfg.trim.speed_mps);
fprintf('\n');
run(fullfile(root, 'src', 'BEMTFLAP_SWITCHED.m'));

save(fullfile(root, 'last_run_results.mat'), 'trim_results');

disp('Trim table columns: V, elevator, collective, longitudinal, lateral, yaw, pitch, roll, power');
disp(trim_results.trim_table);
fprintf('Number of trim speed points run: %d\n', trim_results.n_speed_points);
