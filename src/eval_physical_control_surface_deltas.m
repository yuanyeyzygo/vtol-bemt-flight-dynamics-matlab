function [delta_coeff, physical_deflections] = eval_physical_control_surface_deltas(vehicle_control, alpha_deg, mach, surfaces, control_mapping)
%EVAL_PHYSICAL_CONTROL_SURFACE_DELTAS Sum WL/WR/VL/VR surface increments.
%
% vehicle_control normally contains three channel commands, in degrees:
%   [pitch_channel yaw_channel roll_channel]
%
% A six-channel form is also accepted for future use:
%   [pitch1 pitch2 yaw1 yaw2 roll1 roll2]
%
% By default, the physical deflections follow the current channel-to-surface convention:
%   DVL1 = gain * (yaw1 - pitch1)
%   DVR1 = gain * (yaw1 + pitch1)
%   DVL2 = gain * (yaw2 - pitch2)
%   DVR2 = gain * (yaw2 + pitch2)
%   DWL1 = DWR1 = roll1
%   DWL2 = DWR2 = roll2
%
% A mapping struct can also be passed as the fifth input:
%   mapping.surface_names
%   mapping.channel_names
%   mapping.matrix
%   mapping.bias_deg
%
% delta_coeff returns [dCD dCL dCm dCC dCn dCl].

    if nargin < 5 || isempty(control_mapping)
        control_mapping = 0.5;
    end

    vehicle_control = vehicle_control(:);

    if isstruct(control_mapping)
        surface_names = control_mapping.surface_names;
        M = control_mapping.matrix;
        bias = control_mapping.bias_deg(:);
        if numel(vehicle_control) ~= size(M,2)
            error('ControlSurface:BadControlVector', ...
                'vehicle_control has %d values, but mapping expects %d channels.', ...
                numel(vehicle_control), size(M,2));
        end
        deflection_values = M * vehicle_control + bias;
        physical_deflections = struct();
        for ii = 1:numel(surface_names)
            physical_deflections.(surface_names{ii}) = deflection_values(ii);
        end
    else
        channel_to_physical_gain = control_mapping;
        if numel(vehicle_control) == 3
            pitch1 = vehicle_control(1);
            pitch2 = vehicle_control(1);
            yaw1 = vehicle_control(2);
            yaw2 = vehicle_control(2);
            roll1 = vehicle_control(3);
            roll2 = vehicle_control(3);
        elseif numel(vehicle_control) == 6
            pitch1 = vehicle_control(1);
            pitch2 = vehicle_control(2);
            yaw1 = vehicle_control(3);
            yaw2 = vehicle_control(4);
            roll1 = vehicle_control(5);
            roll2 = vehicle_control(6);
        else
            error('ControlSurface:BadControlVector', ...
                'vehicle_control must contain 3 channel commands or 6 split-channel commands.');
        end

        physical_deflections = struct();
        physical_deflections.VL1 = channel_to_physical_gain * (yaw1 - pitch1);
        physical_deflections.VR1 = channel_to_physical_gain * (yaw1 + pitch1);
        physical_deflections.VL2 = channel_to_physical_gain * (yaw2 - pitch2);
        physical_deflections.VR2 = channel_to_physical_gain * (yaw2 + pitch2);
        physical_deflections.WL1 = roll1;
        physical_deflections.WR1 = roll1;
        physical_deflections.WL2 = roll2;
        physical_deflections.WR2 = roll2;
    end

    delta_coeff = zeros(1,6);
    surface_names = fieldnames(surfaces);
    for ii = 1:numel(surface_names)
        name = surface_names{ii};
        if isfield(physical_deflections, name)
            delta_coeff = delta_coeff + surfaces.(name).eval( ...
                physical_deflections.(name), alpha_deg, mach);
        end
    end
end
