function CREATE_RESPONSE_SIMULINK_MEX_MODEL()
%CREATE_RESPONSE_SIMULINK_MEX_MODEL Build the fast disk-model Simulink file.

root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'src'));
cd(root);

sim_init = SETUP_FAST_DISK_MEX_BASE();
sim_model_local = sim_init.model;

model = 'VTOL_RESPONSE_SIMULINK_MEX';
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
    'StopTime', '2.0', ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', 'dt_sim');
set_param(model, 'InitFcn', 'SETUP_FAST_DISK_MEX_BASE;');

add_block('simulink/Sources/Clock', [model '/Time'], 'Position', [40 45 70 75]);
add_block('simulink/Discrete/Unit Delay', [model '/Aircraft State Memory'], ...
    'InitialCondition', 'x0', 'SampleTime', 'dt_sim', 'Position', [40 120 120 160]);
add_block('simulink/Discrete/Unit Delay', [model '/Rotor State Memory'], ...
    'InitialCondition', 'rotor_state0', 'SampleTime', 'dt_sim', 'Position', [40 205 120 245]);

add_block('simulink/Sources/Constant', [model '/Pilot Stick'], ...
    'Value', 'pilot_stick', 'SampleTime', 'dt_sim', 'Position', [40 290 120 330]);
add_block('simulink/Math Operations/Gain', [model '/Stick Gain Deg'], ...
    'Gain', 'diag(pilot_stick_to_control_gain_deg)', ...
    'Multiplication', 'Matrix(K*u)', ...
    'Position', [160 285 245 335]);
add_block('simulink/Sources/Constant', [model '/Direct Control Debug Delta'], ...
    'Value', 'control_delta', 'SampleTime', 'dt_sim', 'Position', [40 345 120 385]);
add_block('simulink/Math Operations/Sum', [model '/Control Delta'], ...
    'Inputs', '++', 'Position', [285 302 315 353]);

add_block('simulink/Sources/Constant', [model '/Rotor Tilt Angle Deg'], ...
    'Value', 'rotor_tilt_angle_deg', 'SampleTime', 'dt_sim', 'Position', [40 520 140 560]);

add_block('simulink/Sources/Constant', [model '/Fixed Control Debug Delta'], ...
    'Value', 'fixed_control_delta', 'SampleTime', 'dt_sim', 'Position', [40 620 140 660]);

add_block('simulink/Sources/Constant', [model '/dt'], ...
    'Value', 'dt_sim', 'SampleTime', 'dt_sim', 'Position', [40 760 120 795]);

add_block('simulink/Signal Routing/Mux', [model '/Pack Inputs'], ...
    'Inputs', '7', 'Position', [210 130 225 650]);
add_block(sprintf('simulink/User-Defined Functions/Interpreted MATLAB\nFunction'), ...
    [model '/VTOL Response Step MEX'], ...
    'MATLABFcn', 'VTOL_RESPONSE_SIM_VECTOR_MEX', ...
    'OutputDimensions', num2str(response_output_width), ...
    'SampleTime', 'dt_sim', ...
    'Position', [285 235 485 295]);
add_block('simulink/Signal Routing/Demux', [model '/Unpack Outputs'], ...
    'Outputs', demux_widths, 'Position', [555 145 570 410]);
add_block('simulink/Signal Routing/Demux', [model '/Demux'], ...
    'Outputs', '12', 'Position', [655 95 670 420]);

add_block('simulink/Sinks/To Workspace', [model '/x_sim'], ...
    'VariableName', 'x_sim', 'SaveFormat', 'Structure With Time', ...
    'Position', [720 35 815 65]);
add_block('simulink/Sinks/To Workspace', [model '/rotor_state_sim'], ...
    'VariableName', 'rotor_state_sim', 'SaveFormat', 'Structure With Time', ...
    'Position', [720 455 830 485]);
add_block('simulink/Sinks/To Workspace', [model '/forces_sim'], ...
    'VariableName', 'forces_sim', 'SaveFormat', 'Structure With Time', ...
    'Position', [720 510 815 540]);
add_block('simulink/Sinks/To Workspace', [model '/vi_sim'], ...
    'VariableName', 'vi_sim', 'SaveFormat', 'Structure With Time', ...
    'Position', [720 565 815 595]);
add_block('simulink/Sinks/To Workspace', [model '/vi_qs_sim'], ...
    'VariableName', 'vi_qs_sim', 'SaveFormat', 'Structure With Time', ...
    'Position', [720 620 815 650]);
add_block('simulink/Sinks/To Workspace', [model '/tau_sim'], ...
    'VariableName', 'tau_sim', 'SaveFormat', 'Structure With Time', ...
    'Position', [720 675 815 705]);

state_names = {'Ux','Uy','Uz','p','q','r','roll','pitch','yaw','x','y','z'};
for ii = 1:numel(state_names)
    y0 = 85 + (ii-1)*32;
    add_block('simulink/Sinks/Scope', [model '/' state_names{ii}], ...
        'Position', [760 y0 820 y0+22]);
end

add_line(model, 'Time/1', 'Pack Inputs/1', 'autorouting', 'on');
add_line(model, 'Aircraft State Memory/1', 'Pack Inputs/2', 'autorouting', 'on');
add_line(model, 'Rotor State Memory/1', 'Pack Inputs/3', 'autorouting', 'on');
add_line(model, 'Pilot Stick/1', 'Stick Gain Deg/1', 'autorouting', 'on');
add_line(model, 'Stick Gain Deg/1', 'Control Delta/1', 'autorouting', 'on');
add_line(model, 'Direct Control Debug Delta/1', 'Control Delta/2', 'autorouting', 'on');
add_line(model, 'Control Delta/1', 'Pack Inputs/4', 'autorouting', 'on');
add_line(model, 'Rotor Tilt Angle Deg/1', 'Pack Inputs/5', 'autorouting', 'on');
add_line(model, 'Fixed Control Debug Delta/1', 'Pack Inputs/6', 'autorouting', 'on');
add_line(model, 'dt/1', 'Pack Inputs/7', 'autorouting', 'on');
add_line(model, 'Pack Inputs/1', 'VTOL Response Step MEX/1', 'autorouting', 'on');
add_line(model, 'VTOL Response Step MEX/1', 'Unpack Outputs/1', 'autorouting', 'on');

add_line(model, 'Unpack Outputs/1', 'Aircraft State Memory/1', 'autorouting', 'on');
add_line(model, 'Unpack Outputs/2', 'Rotor State Memory/1', 'autorouting', 'on');
add_line(model, 'Unpack Outputs/1', 'x_sim/1', 'autorouting', 'on');
add_line(model, 'Unpack Outputs/1', 'Demux/1', 'autorouting', 'on');
add_line(model, 'Unpack Outputs/2', 'rotor_state_sim/1', 'autorouting', 'on');
add_line(model, 'Unpack Outputs/3', 'forces_sim/1', 'autorouting', 'on');
add_line(model, 'Unpack Outputs/4', 'vi_sim/1', 'autorouting', 'on');
add_line(model, 'Unpack Outputs/5', 'vi_qs_sim/1', 'autorouting', 'on');
add_line(model, 'Unpack Outputs/6', 'tau_sim/1', 'autorouting', 'on');
for ii = 1:numel(state_names)
    add_line(model, sprintf('Demux/%d', ii), [state_names{ii} '/1'], 'autorouting', 'on');
end

Simulink.BlockDiagram.arrangeSystem(model);
save_system(model, model_file);
fprintf('\n=== Fast disk MEX Simulink model created ===\n%s\n', model_file);
end
