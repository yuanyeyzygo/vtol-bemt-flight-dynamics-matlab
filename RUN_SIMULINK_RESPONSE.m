if exist('flap_model_override', 'var')
    requested_flap_model = string(flap_model_override);
else
    requested_flap_model = "";
end
clearvars -except requested_flap_model;
clc;

root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'src'));
cd(root);

if requested_flap_model ~= ""
    flap_model_override = requested_flap_model;
end
RUN_RESPONSE_SIMULINK_SETUP;

model = 'VTOL_RESPONSE_SIMULINK';
if exist([model '.slx'], 'file') == 0
    if requested_flap_model ~= ""
        CREATE_RESPONSE_SIMULINK_MODEL(requested_flap_model);
    else
        CREATE_RESPONSE_SIMULINK_MODEL;
    end
end

load_system(model);
out = sim(model);

fprintf('\n=== Simulink response run complete ===\n');
fprintf('Stop time: %s s\n', get_param(model, 'StopTime'));
fprintf('Outputs in SimulationOutput: x_sim, rotor_state_sim, forces_sim, vi_sim, vi_qs_sim, tau_sim\n');
