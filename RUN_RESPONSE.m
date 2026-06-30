clear;
clc;

root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'src'));
data_dir = fullfile(root, 'data');

% -------------------------------------------------------------------------
% Nonlinear response example
% -------------------------------------------------------------------------
% This file trims one flight condition, then runs a nonlinear time response.
% The response model recalculates all rotor and fuselage forces at every
% aircraft RK4 sub-step. Rotor flap integration uses the switch below.

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
cfg.data.aero.base_sheets = {};
cfg.data.aero.base_sheet_tilt_angle_deg = [];
cfg.data.aero.control_surface_sheets = {'WL1','WL2','WR1','WR2','VL1','VL2','VR1','VR2'};
cfg.data.chord.txt_file = 'Chord.txt';
cfg.data.chord.mat_var = 'F';
cfg.data.pretwist.txt_file = 'Pretwist.txt';
cfg.data.pretwist.mat_var = 'pre_twist';

cfg.environment.rho_kg_m3 = 1.225;
cfg.environment.gravity_m_s2 = 9.81;
cfg.environment.use_isa = false;
cfg.environment.initial_altitude_m = 0;
cfg.environment.altitude_min_m = -500;
cfg.environment.altitude_max_m = 20000;
cfg.vehicle.mass_kg = 1900;
cfg.vehicle.inertia_kg_m2 = [1966.5, 5245.3, 3282.7];

cfg.rotor.radius_m = 1.3;
cfg.rotor.blade_count = 5;
cfg.rotor.omega_rad_s = 90;
cfg.rotor.flap_inertia_kg_m2 = 2.25;
cfg.rotor.flap_spring_nm_rad = 16000;
cfg.rotor.airfoil_section_edges = [0.25 0.40 0.50 0.80 0.92];
cfg.rotor.rotational_direction = [1 -1 1 -1 1 -1];
cfg.rotor.blade_element_count = 10;
cfg.rotor.azimuth_steps = 72;
cfg.rotor.flap_integrator = "rk4";
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
cfg.trim.max_iterations = 10000;
cfg.trim.tol = 1e-8;
cfg.stability.max_iterations = 20;
cfg.stability.tol = 1e-8;
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
cfg.response.duration_s = 3;
cfg.response.dt_s = 0.05;
cfg.response.aircraft_integrator = "rk4";
cfg.response.update_rotor_states = true;
cfg.response.initial_state_delta = zeros(12,1);       % [u v w p q r phi theta psi x y z]
cfg.response.control_delta = [0.5; 0; 0; 0];          % [collective longitudinal lateral yaw], deg
cfg.response.rotor_tilt_angle_deg = [];               % empty -> use trim rotor tilt angles, deg
cfg.response.fixed_wing_control_delta = zeros(3,1);  % [elevator rudder aileron], deg
cfg.response.step_time_s = 1.0;

fprintf('Running nonlinear response: tilt %.6g deg, speed %.6g m/s\n', ...
    cfg.trim.tilt_angle_deg, cfg.trim.speed_mps(1));
run(fullfile(root, 'src', 'BEMTFLAP_SWITCHED.m'));

save(fullfile(root, 'last_response_results.mat'), 'trim_results');

disp('Response state order:');
disp(trim_results.response.state_order);
disp('Final response state:');
disp(trim_results.response.final_state.');
