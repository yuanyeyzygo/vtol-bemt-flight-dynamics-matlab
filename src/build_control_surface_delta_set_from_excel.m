function surfaces = build_control_surface_delta_set_from_excel(fname, sheetNames, extrapMode, clampToRange)
%BUILD_CONTROL_SURFACE_DELTA_SET_FROM_EXCEL Load WL/WR/VL/VR sheets.
%
% Preferred sheet columns:
%   deflection_deg alpha_deg dCD dCL dCm dCC dCn dCl
%
% Legacy sheets with optional Mach are also accepted:
%   deflection_deg Mach alpha_deg dCD dCL dCm dCC dCn dCl

    if nargin < 2 || isempty(sheetNames)
        sheetNames = {'WL1', 'WL2', 'WR1', 'WR2', 'VL1', 'VL2', 'VR1', 'VR2'};
    end
    if nargin < 3 || isempty(extrapMode), extrapMode = 'linear'; end
    if nargin < 4 || isempty(clampToRange), clampToRange = false; end

    persistent surface_cache
    if isempty(surface_cache)
        surface_cache = containers.Map('KeyType', 'char', 'ValueType', 'any');
    end
    cache_key = make_excel_surface_cache_key(fname, sheetNames, extrapMode, clampToRange);
    if isKey(surface_cache, cache_key)
        surfaces = surface_cache(cache_key);
        return;
    end

    surfaces = struct();
    for ii = 1:numel(sheetNames)
        name = char(sheetNames{ii});
        surfaces.(name) = build_control_surface_delta_lookup_from_excel( ...
            fname, name, extrapMode, clampToRange);
    end
    surface_cache(cache_key) = surfaces;
end

function key = make_excel_surface_cache_key(fname, sheetNames, extrapMode, clampToRange)
    info = dir(fname);
    if isempty(info)
        error('AeroExcel:MissingFile', 'Excel aero database not found: %s', fname);
    end
    file_id = fname;
    if isfield(info, 'folder') && ~isempty(info.folder)
        file_id = fullfile(info.folder, info.name);
    end
    if isstring(sheetNames)
        sheet_key = strjoin(cellstr(sheetNames(:).'), '|');
    else
        sheet_key = strjoin(cellfun(@char, sheetNames(:).', 'UniformOutput', false), '|');
    end
    key = sprintf('%s|%s|%s|%d|%.17g|%d', ...
        file_id, sheet_key, char(extrapMode), logical(clampToRange), ...
        info.datenum, info.bytes);
end

function out = build_control_surface_delta_lookup_from_excel(fname, sheetName, extrapMode, clampToRange)
    A = read_control_surface_rows(fname, sheetName);

    deflection = A(:,1);
    mach = A(:,2);
    alpha = A(:,3);
    coeffNames = {'dCD', 'dCL', 'dCm', 'dCC', 'dCn', 'dCl'};
    coeff = A(:,4:9);

    A = [deflection(:), mach(:), alpha(:), coeff];
    if any(~isfinite(A(:)))
        error('AeroExcel:BadControlSurface', 'Non-finite values found in %s sheet %s.', fname, sheetName);
    end

    deflectionGrid = unique(deflection, 'sorted');
    machGrid = unique(mach, 'sorted');
    alphaGrid = unique(alpha, 'sorted');

    if numel(machGrid) == 1
        F = build_2d_interpolants(deflection, alpha, coeff, deflectionGrid, alphaGrid, extrapMode);
    else
        F = build_3d_interpolants(deflection, mach, alpha, coeff, deflectionGrid, machGrid, alphaGrid, extrapMode);
    end

    out = struct();
    out.type = "excel_control_surface";
    out.file = fname;
    out.sheet = sheetName;
    out.deflectionGrid = deflectionGrid;
    out.machGrid = machGrid;
    out.alphaGrid = alphaGrid;
    out.coeffNames = coeffNames;
    out.F = F;
    out.eval = @(deflectionQuery, alphaQuery, machQuery) eval_surface_delta( ...
        F, deflectionQuery, alphaQuery, machQuery, deflectionGrid, machGrid, alphaGrid, clampToRange);
    out.eval2 = @(deflectionQuery, alphaQuery) out.eval(deflectionQuery, alphaQuery, machGrid(1));
end

function A = read_control_surface_rows(fname, sheetName)
    raw = readmatrix(fname, 'Sheet', sheetName);
    if isempty(raw)
        error('AeroExcel:BadControlSurface', 'Sheet %s is empty.', sheetName);
    end

    raw = raw(:, any(isfinite(raw), 1));
    raw = raw(all(isfinite(raw), 2), :);
    if isempty(raw)
        error('AeroExcel:BadControlSurface', ...
            'Sheet %s has no complete numeric rows.', sheetName);
    end

    if size(raw, 2) >= 9
        A = raw(:, 1:9);
        return;
    end

    if size(raw, 2) >= 8
        mach = zeros(size(raw, 1), 1);
        A = [raw(:, 1), mach, raw(:, 2:8)];
        return;
    end

    error('AeroExcel:BadControlSurface', ...
        ['Sheet %s must contain either [deflection alpha dCD dCL dCm dCC dCn dCl] ' ...
         'or [deflection Mach alpha dCD dCL dCm dCC dCn dCl].'], sheetName);
end

function F = build_2d_interpolants(deflection, alpha, coeff, deflectionGrid, alphaGrid, extrapMode)
    nD = numel(deflectionGrid);
    nA = numel(alphaGrid);
    F = cell(1,6);

    for k = 1:6
        grid = nan(nD, nA);
        for i = 1:nD
            for j = 1:nA
                idx = (deflection == deflectionGrid(i)) & (alpha == alphaGrid(j));
                if nnz(idx) ~= 1
                    error('AeroExcel:BadControlGrid', ...
                        'Grid point (deflection=%g, alpha=%g) has %d entries.', ...
                        deflectionGrid(i), alphaGrid(j), nnz(idx));
                end
                grid(i,j) = coeff(idx,k);
            end
        end
        F{k} = griddedInterpolant({deflectionGrid, alphaGrid}, grid, 'linear', extrapMode);
    end
end

function F = build_3d_interpolants(deflection, mach, alpha, coeff, deflectionGrid, machGrid, alphaGrid, extrapMode)
    nD = numel(deflectionGrid);
    nM = numel(machGrid);
    nA = numel(alphaGrid);
    F = cell(1,6);

    for k = 1:6
        grid = nan(nD, nM, nA);
        for i = 1:nD
            for m = 1:nM
                for j = 1:nA
                    idx = (deflection == deflectionGrid(i)) & ...
                          (mach == machGrid(m)) & ...
                          (alpha == alphaGrid(j));
                    if nnz(idx) ~= 1
                        error('AeroExcel:BadControlGrid', ...
                            'Grid point (deflection=%g, Mach=%g, alpha=%g) has %d entries.', ...
                            deflectionGrid(i), machGrid(m), alphaGrid(j), nnz(idx));
                    end
                    grid(i,m,j) = coeff(idx,k);
                end
            end
        end
        F{k} = griddedInterpolant({deflectionGrid, machGrid, alphaGrid}, grid, 'linear', extrapMode);
    end
end

function vals = eval_surface_delta(F, deflectionQuery, alphaQuery, machQuery, deflectionGrid, machGrid, alphaGrid, clampToRange)
    if nargin < 4 || isempty(machQuery), machQuery = machGrid(1); end
    if clampToRange
        deflectionQuery = clamp_value(deflectionQuery, deflectionGrid(1), deflectionGrid(end));
        machQuery = clamp_value(machQuery, machGrid(1), machGrid(end));
        alphaQuery = clamp_value(alphaQuery, alphaGrid(1), alphaGrid(end));
    end

    vals = zeros(1,6);
    if numel(machGrid) == 1
        for k = 1:6
            vals(k) = F{k}(deflectionQuery, alphaQuery);
        end
    else
        for k = 1:6
            vals(k) = F{k}(deflectionQuery, machQuery, alphaQuery);
        end
    end
end
function y = clamp_value(x, lo, hi)
    y = min(max(x, lo), hi);
end
