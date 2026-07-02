if exist('flap_model_override', 'var')
    requested_flap_model = string(flap_model_override);
else
    requested_flap_model = "";
end
if exist('response_dt_s_override', 'var')
    requested_response_dt_s = response_dt_s_override;
else
    requested_response_dt_s = [];
end
if exist('sim_setup_overrides', 'var')
    requested_sim_setup_overrides = sim_setup_overrides;
else
    requested_sim_setup_overrides = struct();
end
clearvars -except requested_flap_model requested_response_dt_s requested_sim_setup_overrides;
clc;

root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'src'));
cd(root);

if requested_flap_model ~= ""
    flap_model_override = requested_flap_model;
end
if ~isempty(requested_response_dt_s)
    response_dt_s_override = requested_response_dt_s;
end
if ~isfield(requested_sim_setup_overrides, 'response') || ~isstruct(requested_sim_setup_overrides.response)
    requested_sim_setup_overrides.response = struct();
end
requested_sim_setup_overrides.response.compile_mex = false;
sim_setup_overrides = requested_sim_setup_overrides;
RUN_RESPONSE_SIMULINK_SETUP;
model = 'VTOL_RESPONSE_SIMULINK';
CREATE_RESPONSE_SIMULINK_MODEL;

load_system(model);
out = sim(model);

fprintf('\n=== Simulink response run complete ===\n');
fprintf('Stop time: %s s\n', get_param(model, 'StopTime'));
fprintf('Outputs in SimulationOutput: x_sim, rotor_state_sim, forces_sim, vi_sim, vi_qs_sim, tau_sim\n');
