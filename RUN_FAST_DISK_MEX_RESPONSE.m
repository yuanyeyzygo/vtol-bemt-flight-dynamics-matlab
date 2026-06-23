function out = RUN_FAST_DISK_MEX_RESPONSE(stop_time_s)
%RUN_FAST_DISK_MEX_RESPONSE Run the fast default/disk/MEX Simulink response.
%
% Usage:
%   out = RUN_FAST_DISK_MEX_RESPONSE;
%   out = RUN_FAST_DISK_MEX_RESPONSE(1.5);
%
% Run RUN_RESPONSE_SIMULINK_SETUP before this function. This runner uses the
% current base-workspace sim_init and does not load cached init MAT files.

if nargin < 1 || isempty(stop_time_s)
    stop_time_s = 1.5;
end

root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'src'));
cd(root);

SETUP_FAST_DISK_MEX_BASE;

model = 'VTOL_RESPONSE_SIMULINK_MEX';
if exist([model '.slx'], 'file') == 0
    CREATE_RESPONSE_SIMULINK_MEX_MODEL;
end

load_system(model);
set_param(model, 'StopTime', num2str(stop_time_s, '%.15g'));
out = sim(model);
assignin('base', 'out', out);

fprintf('\n=== Fast disk MEX response complete ===\n');
fprintf('Model: %s\n', model);
fprintf('Stop time: %.6g s\n', stop_time_s);
fprintf('Sample time: %.6g s\n', evalin('base', 'dt_sim'));
fprintf('Outputs: out.x_sim, out.rotor_state_sim, out.forces_sim, out.vi_sim, out.vi_qs_sim, out.tau_sim\n');
end
