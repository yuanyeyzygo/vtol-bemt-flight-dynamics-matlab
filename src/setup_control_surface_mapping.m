function mapping = setup_control_surface_mapping(controls_cfg, surfaces)
%SETUP_CONTROL_SURFACE_MAPPING Build channel-to-physical-surface mapping.
%
% Default 3-channel input order:
%   [pitch yaw roll]
%
% Default surface order:
%   WL1 WL2 WR1 WR2 VL1 VL2 VR1 VR2
%
% Users can override:
%   cfg.controls.channel_names
%   cfg.controls.physical_surface_names
%   cfg.controls.surface_mixing_matrix
%   cfg.controls.surface_bias_deg

    defaultNames = {'WL1','WL2','WR1','WR2','VL1','VL2','VR1','VR2'};
    if nargin >= 2 && ~isempty(surfaces)
        surfaceNames = fieldnames(surfaces).';
        [~, order] = ismember(defaultNames, surfaceNames);
        if all(order > 0)
            surfaceNames = defaultNames;
        end
    else
        surfaceNames = defaultNames;
    end

    channelNames = {'pitch','yaw','roll'};
    gain = 0.5;
    if isfield(controls_cfg, 'channel_to_physical_gain')
        gain = controls_cfg.channel_to_physical_gain;
    end

    M = default_surface_mixing_matrix(surfaceNames, channelNames, gain);
    bias = zeros(numel(surfaceNames), 1);

    if isfield(controls_cfg, 'channel_names') && ~isempty(controls_cfg.channel_names)
        channelNames = cellstr(string(controls_cfg.channel_names(:).'));
    end
    if isfield(controls_cfg, 'physical_surface_names') && ~isempty(controls_cfg.physical_surface_names)
        surfaceNames = cellstr(string(controls_cfg.physical_surface_names(:).'));
    end
    if isfield(controls_cfg, 'surface_mixing_matrix') && ~isempty(controls_cfg.surface_mixing_matrix)
        M = controls_cfg.surface_mixing_matrix;
    else
        M = default_surface_mixing_matrix(surfaceNames, channelNames, gain);
    end
    if isfield(controls_cfg, 'surface_bias_deg') && ~isempty(controls_cfg.surface_bias_deg)
        bias = controls_cfg.surface_bias_deg(:);
    else
        bias = zeros(numel(surfaceNames), 1);
    end

    if size(M,1) ~= numel(surfaceNames)
        error('ControlSurface:BadMapping', ...
            'cfg.controls.surface_mixing_matrix must have one row per physical surface.');
    end
    if size(M,2) ~= numel(channelNames)
        error('ControlSurface:BadMapping', ...
            'cfg.controls.surface_mixing_matrix must have one column per control channel.');
    end
    if numel(bias) ~= numel(surfaceNames)
        error('ControlSurface:BadMapping', ...
            'cfg.controls.surface_bias_deg must have one value per physical surface.');
    end

    mapping = struct();
    mapping.surface_names = surfaceNames;
    mapping.channel_names = channelNames;
    mapping.matrix = M;
    mapping.bias_deg = bias;
end

function M = default_surface_mixing_matrix(surfaceNames, channelNames, gain)
    M = zeros(numel(surfaceNames), numel(channelNames));
    for i = 1:numel(surfaceNames)
        s = upper(string(surfaceNames{i}));
        for j = 1:numel(channelNames)
            c = lower(string(channelNames{j}));
            switch s
                case {"WL1","WR1"}
                    if c == "roll" || c == "roll1"
                        M(i,j) = 1;
                    end
                case {"WL2","WR2"}
                    if c == "roll" || c == "roll2"
                        M(i,j) = 1;
                    end
                case "VL1"
                    if c == "pitch" || c == "pitch1"
                        M(i,j) = -gain;
                    elseif c == "yaw" || c == "yaw1"
                        M(i,j) = gain;
                    end
                case "VR1"
                    if c == "pitch" || c == "pitch1"
                        M(i,j) = gain;
                    elseif c == "yaw" || c == "yaw1"
                        M(i,j) = gain;
                    end
                case "VL2"
                    if c == "pitch" || c == "pitch2"
                        M(i,j) = -gain;
                    elseif c == "yaw" || c == "yaw2"
                        M(i,j) = gain;
                    end
                case "VR2"
                    if c == "pitch" || c == "pitch2"
                        M(i,j) = gain;
                    elseif c == "yaw" || c == "yaw2"
                        M(i,j) = gain;
                    end
            end
        end
    end
end
