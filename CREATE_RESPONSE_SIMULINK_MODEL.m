function CREATE_RESPONSE_SIMULINK_MODEL(flap_model)
%CREATE_RESPONSE_SIMULINK_MODEL Build a runnable nonlinear response Simulink model.
%
% The model uses RUN_RESPONSE_SIMULINK_SETUP.m to trim first, then advances
% [aircraft state; rotor periodic states; uniform inflow] with one discrete
% response block per sample time.

root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'src'));
cd(root);

if nargin >= 1 && ~isempty(flap_model)
    flap_model_override = string(flap_model); %#ok<NASGU>
end
RUN_RESPONSE_SIMULINK_SETUP;
sim_model_local = evalin('base', 'sim_model');
model_flap_model = string(sim_model_local.flap_model);

model = 'VTOL_RESPONSE_SIMULINK';
model_file = fullfile(root, [model '.slx']);
if bdIsLoaded(model)
    set_param(model, 'Dirty', 'off');
    close_system(model, 0);
end
if exist(model_file, 'file') == 2
    delete(model_file);
end

n_rotor_state = sim_model_local.n_rotors * sim_model_local.rotor_state_size;
response_output_width = 12 + n_rotor_state + 6 + 6 + 6 + 6;
demux_widths = sprintf('[12 %d 6 6 6 6]', n_rotor_state);

new_system(model);

set_param(model, ...
    'StopTime', '1.5', ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', 'dt_sim');

init_cmd = "if evalin('base','exist(''sim_model'',''var'')') == 0; flap_model_override='" + model_flap_model + "'; RUN_RESPONSE_SIMULINK_SETUP; end";
set_param(model, 'InitFcn', char(init_cmd));

add_block('simulink/Sources/Clock', [model '/Time'], 'Position', [40 45 70 75]);
add_block('simulink/Discrete/Unit Delay', [model '/Aircraft State Memory'], ...
    'InitialCondition', 'x0', 'SampleTime', 'dt_sim', 'Position', [40 120 120 160]);
add_block('simulink/Discrete/Unit Delay', [model '/Rotor State Memory'], ...
    'InitialCondition', 'rotor_state0', 'SampleTime', 'dt_sim', 'Position', [40 205 120 245]);
add_block('simulink/Sources/Constant', [model '/Rotor Control Delta'], ...
    'Value', 'control_delta', 'SampleTime', 'dt_sim', 'Position', [40 290 120 330]);
add_block('simulink/Sources/Constant', [model '/Rotor Tilt Angle Deg'], ...
    'Value', 'rotor_tilt_angle_deg', 'SampleTime', 'dt_sim', 'Position', [40 355 120 395]);
add_block('simulink/Sources/Constant', [model '/Fixed Wing Control Delta'], ...
    'Value', 'fixed_control_delta', 'SampleTime', 'dt_sim', 'Position', [40 430 120 470]);
add_block('simulink/Sources/Constant', [model '/dt'], ...
    'Value', 'dt_sim', 'SampleTime', 'dt_sim', 'Position', [40 505 120 540]);

add_block('simulink/Signal Routing/Mux', [model '/Pack Inputs'], ...
    'Inputs', '7', 'Position', [190 130 205 470]);

add_block(sprintf('simulink/User-Defined Functions/Interpreted MATLAB\nFunction'), [model '/VTOL Response Step'], ...
    'MATLABFcn', 'VTOL_RESPONSE_SIM_VECTOR', ...
    'OutputDimensions', num2str(response_output_width), ...
    'SampleTime', 'dt_sim', ...
    'Position', [270 235 450 295]);

add_block('simulink/Signal Routing/Demux', [model '/Unpack Outputs'], ...
    'Outputs', demux_widths, 'Position', [520 145 535 410]);

add_block('simulink/Sinks/To Workspace', [model '/x_sim'], ...
    'VariableName', 'x_sim', 'SaveFormat', 'Structure With Time', ...
    'Position', [650 120 745 150]);
add_block('simulink/Sinks/To Workspace', [model '/rotor_state_sim'], ...
    'VariableName', 'rotor_state_sim', 'SaveFormat', 'Structure With Time', ...
    'Position', [650 180 760 210]);
add_block('simulink/Sinks/To Workspace', [model '/forces_sim'], ...
    'VariableName', 'forces_sim', 'SaveFormat', 'Structure With Time', ...
    'Position', [650 240 745 270]);
add_block('simulink/Sinks/To Workspace', [model '/vi_sim'], ...
    'VariableName', 'vi_sim', 'SaveFormat', 'Structure With Time', ...
    'Position', [650 300 745 330]);
add_block('simulink/Sinks/To Workspace', [model '/vi_qs_sim'], ...
    'VariableName', 'vi_qs_sim', 'SaveFormat', 'Structure With Time', ...
    'Position', [650 360 745 390]);
add_block('simulink/Sinks/To Workspace', [model '/tau_sim'], ...
    'VariableName', 'tau_sim', 'SaveFormat', 'Structure With Time', ...
    'Position', [650 420 745 450]);

add_line(model, 'Time/1', 'Pack Inputs/1', 'autorouting', 'on');
add_line(model, 'Aircraft State Memory/1', 'Pack Inputs/2', 'autorouting', 'on');
add_line(model, 'Rotor State Memory/1', 'Pack Inputs/3', 'autorouting', 'on');
add_line(model, 'Rotor Control Delta/1', 'Pack Inputs/4', 'autorouting', 'on');
add_line(model, 'Rotor Tilt Angle Deg/1', 'Pack Inputs/5', 'autorouting', 'on');
add_line(model, 'Fixed Wing Control Delta/1', 'Pack Inputs/6', 'autorouting', 'on');
add_line(model, 'dt/1', 'Pack Inputs/7', 'autorouting', 'on');
add_line(model, 'Pack Inputs/1', 'VTOL Response Step/1', 'autorouting', 'on');
add_line(model, 'VTOL Response Step/1', 'Unpack Outputs/1', 'autorouting', 'on');

add_line(model, 'Unpack Outputs/1', 'Aircraft State Memory/1', 'autorouting', 'on');
add_line(model, 'Unpack Outputs/2', 'Rotor State Memory/1', 'autorouting', 'on');
add_line(model, 'Unpack Outputs/1', 'x_sim/1', 'autorouting', 'on');
add_line(model, 'Unpack Outputs/2', 'rotor_state_sim/1', 'autorouting', 'on');
add_line(model, 'Unpack Outputs/3', 'forces_sim/1', 'autorouting', 'on');
add_line(model, 'Unpack Outputs/4', 'vi_sim/1', 'autorouting', 'on');
add_line(model, 'Unpack Outputs/5', 'vi_qs_sim/1', 'autorouting', 'on');
add_line(model, 'Unpack Outputs/6', 'tau_sim/1', 'autorouting', 'on');

Simulink.BlockDiagram.arrangeSystem(model);
save_system(model, model_file);
fprintf('\n=== Simulink model created ===\n%s\n', model_file);
end
