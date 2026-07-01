function sim_init = SETUP_FAST_DISK_MEX_BASE()
%SETUP_FAST_DISK_MEX_BASE Publish current setup results for Simulink.
%
% Run RUN_RESPONSE_SIMULINK_SETUP first. This helper only republishes the
% current base-workspace sim_init values; it does not load cached init files
% or rebuild the model from old settings.

root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'src'));
cd(root);

if evalin('base', 'exist(''sim_init'', ''var'')') ~= 1
    error('SETUP_FAST_DISK_MEX_BASE:MissingSetup', ...
        ['Run RUN_RESPONSE_SIMULINK_SETUP first. This function no longer ' ...
         'loads cached initialization files.']);
end

sim_init = evalin('base', 'sim_init');
sim_init.model.disk_state_substeps = 8;
sim_init.model = make_mex_compatible_sim_model(sim_init.model, false);

assignin('base', 'sim_init', sim_init);
assignin('base', 'x0', sim_init.x0);
assignin('base', 'rotor_state0', sim_init.rotor_state0);
assignin('base', 'control_trim', sim_init.control_trim_vector);
assignin('base', 'pilot_stick', sim_init.pilot_stick(:));
assignin('base', 'pilot_stick_to_control_gain_deg', sim_init.pilot_stick_to_control_gain_deg(:));
assignin('base', 'control_delta', sim_init.control_delta(:));
assignin('base', 'rotor_tilt_angle_deg', sim_init.rotor_tilt_angle_deg(:));
assignin('base', 'fixed_control_delta', sim_init.fixed_control_delta(:));
assignin('base', 'dt_sim', sim_init.dt_s);
assignin('base', 'sim_model', sim_init.model);
end
