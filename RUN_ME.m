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

data_mode = "default";  % "default" or "lookup"

cfg = struct();

% Data source switches
cfg.switch.geometry = data_mode;   % CG_positions.txt or defaults
cfg.switch.rotor_positions = data_mode; % Rotor_positions.txt or analytic defaults
cfg.switch.chord = data_mode;      % Chord.txt or constant chord
cfg.switch.pretwist = data_mode;   % Pretwist.txt or linear twist
cfg.switch.airfoil = data_mode;    % CS*_cl/cd.txt or default CL/CD formulas
cfg.switch.fuselage = data_mode;   % fuselage coefficient tables or defaults
cfg.switch.controls = data_mode;   % elevator/rudder/aileron tables or defaults
cfg.data.geometry.cg_file = 'CG_positions.txt';
cfg.data.geometry.rotor_positions_file = 'Rotor_positions.txt';
cfg.data.fuselage.reference_point_mm = [3600 0 0];  % V11 aero moment reference point, mm
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
cfg.rotor.radius_m = 1.3;
cfg.rotor.blade_count = 5;
cfg.rotor.omega_rad_s = 90;
cfg.rotor.flap_inertia_kg_m2 = 2.25;
cfg.rotor.flap_spring_nm_rad = 1000;
cfg.rotor.airfoil_section_edges = [0.25 0.40 0.50 0.80 0.92];
cfg.rotor.rotational_direction = [1 -1 1 -1 1 -1]; % +1 or -1 for rotors 1..6
cfg.rotor.blade_element_count = 10;
cfg.rotor.azimuth_steps = 72;

% Fuselage/aero database reference geometry
cfg.fuselage.reference_area_m2 = 12;
cfg.fuselage.mean_aero_chord_m = 1;
cfg.fuselage.span_m = 10;

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
cfg.initial.fixed_wing_control = [0 0 0]; % [elevator rudder aileron]

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
cfg.defaults.geometry.x_cg_mm = 3000;
cfg.defaults.geometry.y_cg_mm = 0;
cfg.defaults.geometry.z_cg_mm = -950;
cfg.defaults.geometry.first_rotor_x_mm = 430
cfg.defaults.geometry.first_rotor_z_mm = -1900;
% Default rotor-position formula coefficients, one row per rotor:
% [x0 x_cos x_sin y z0 z_cos z_sin], mm, where
% x = x0 + x_cos*cos(tilt) + x_sin*sin(tilt)
% z = z0 + z_cos*cos(tilt) + z_sin*sin(tilt)
cfg.defaults.geometry.rotor_position_coeffs_mm = [
    3600 -700    0  6000 -1290    0 -750
    1490 -400 -140  2500 -1490  140 -450
    1490 -400 -140 -2500 -1490  140 -450
    3600 -700    0 -6000 -1290    0 -750
    5710 -400  140  2500 -1210 -140 -400
    5710 -400  140 -2500 -1210 -140 -400];

cfg.defaults.chord_m = 0.18;
cfg.defaults.pretwist_root_deg = 0;
cfg.defaults.pretwist_tip_deg = -12;

cfg.defaults.airfoil.cl_alpha_per_rad = 2*pi;
cfg.defaults.airfoil.cl_max = 1.45;
cfg.defaults.airfoil.cd0 = 0.012;
cfg.defaults.airfoil.cd_alpha2 = 0.08;

cfg.defaults.fuselage.cd0 = 0.08;
cfg.defaults.fuselage.cl_alpha_per_rad = 2.5;
cfg.defaults.fuselage.cm_alpha_per_rad = -0.15;
cfg.defaults.fuselage.cc_beta = -0.7;
cfg.defaults.fuselage.cn_beta = 0.08;
cfg.defaults.fuselage.cll_beta = -0.08;

cfg.defaults.controls.elevator = [0, 0.40, -0.70]; % [dCD dCL dCM] per rad
cfg.defaults.controls.rudder = [0.25, -0.08, 0];   % [dCC dCN dCLL] per rad
cfg.defaults.controls.aileron = [0, 0, 0.12];      % [dCC dCN dCLL] per rad

fprintf('Requested trim speeds [m/s]:');
fprintf(' %.6g', cfg.trim.speed_mps);
fprintf('\n');
run(fullfile(root, 'src', 'BEMTFLAP_SWITCHED.m'));

save(fullfile(root, 'last_run_results.mat'), 'trim_results');

disp('Trim table columns: V, elevator, collective, longitudinal, lateral, yaw, pitch, roll, power');
disp(trim_results.trim_table);
fprintf('Number of trim speed points run: %d\n', trim_results.n_speed_points);
