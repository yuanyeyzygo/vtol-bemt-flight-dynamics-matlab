function surfaces = build_control_surface_delta_set(data_dir, extrapMode, clampToRange)
%BUILD_CONTROL_SURFACE_DELTA_SET Load WL/WR/VL/VR physical-surface tables.

    if nargin < 1 || isempty(data_dir), data_dir = 'data'; end
    if nargin < 2 || isempty(extrapMode), extrapMode = 'linear'; end
    if nargin < 3 || isempty(clampToRange), clampToRange = false; end

    names = {'WL1', 'WL2', 'WR1', 'WR2', 'VL1', 'VL2', 'VR1', 'VR2'};
    surfaces = struct();
    for i = 1:numel(names)
        name = names{i};
        fname = fullfile(data_dir, [name '.txt']);
        if exist(fname, 'file') ~= 2
            error('ControlSurface:MissingFile', ...
                'Control surface lookup file not found: %s', fname);
        end
        surfaces.(name) = build_control_surface_delta_lookup(fname, extrapMode, clampToRange);
    end
end
