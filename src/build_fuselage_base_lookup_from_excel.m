function [cd, cl, cm, cc, cn, cll] = build_fuselage_base_lookup_from_excel(fname, sheetName, extrapMode, clampToRange, sheetTiltAngles)
%BUILD_FUSELAGE_BASE_LOOKUP_FROM_EXCEL Build base-aero lookups from Excel.
%
% Supported formats:
%
% Preferred single sheet columns:
%   tilt_angle_deg beta_deg alpha_deg CD CL Cm CC Cn Cl
%
% Legacy single sheet columns with optional Mach:
%   tilt_angle_deg beta_deg Mach alpha_deg CD CL Cm CC Cn Cl
%
% Split sheets, one nacelle/airframe tilt angle per sheet:
%   beta_deg alpha_deg CD CL Cm CC Cn Cl
%
% Legacy split sheets with optional Mach:
%   beta_deg Mach alpha_deg CD CL Cm CC Cn Cl
%
% For split sheets, provide sheetName as a cell/string array and pass the
% corresponding sheetTiltAngles vector, or use sheet names containing the
% tilt angle such as base_0, base_30, base_60, base_90.
%
% The returned coefficient objects evaluate absolute coefficients as a
% function of tilt angle, sideslip beta, and angle of attack. Mach is kept
% only for backward-compatible Excel files that already contain it.

    if nargin < 2 || isempty(sheetName), sheetName = 'base_aero'; end
    if nargin < 3 || isempty(extrapMode), extrapMode = 'linear'; end
    if nargin < 4 || isempty(clampToRange), clampToRange = false; end
    if nargin < 5, sheetTiltAngles = []; end

    persistent lookup_cache
    if isempty(lookup_cache)
        lookup_cache = containers.Map('KeyType', 'char', 'ValueType', 'any');
    end
    cache_key = make_excel_cache_key(fname, sheetName, extrapMode, clampToRange, sheetTiltAngles);
    if isKey(lookup_cache, cache_key)
        cached = lookup_cache(cache_key);
        cd = cached{1};
        cl = cached{2};
        cm = cached{3};
        cc = cached{4};
        cn = cached{5};
        cll = cached{6};
        return;
    end

    [A, sheetLabel] = read_base_aero_rows(fname, sheetName, sheetTiltAngles);

    tilt = A(:,1);
    beta = A(:,2);
    mach = A(:,3);
    alpha = A(:,4);
    coeffNames = {'CD', 'CL', 'Cm', 'CC', 'Cn', 'Cl'};
    coeff = A(:,5:10);

    A = [tilt(:), beta(:), mach(:), alpha(:), coeff];
    if any(~isfinite(A(:)))
        error('AeroExcel:BadBaseAero', 'Non-finite values found in %s sheet(s) %s.', fname, sheetLabel);
    end

    tiltGrid = unique(tilt, 'sorted');
    betaGrid = unique(beta, 'sorted');
    machGrid = unique(mach, 'sorted');
    alphaGrid = unique(alpha, 'sorted');

    if numel(machGrid) == 1
        F = build_base_interpolants_3d(tilt, beta, alpha, coeff, ...
            tiltGrid, betaGrid, alphaGrid, extrapMode);
    else
        F = build_base_interpolants_4d(tilt, beta, mach, alpha, coeff, ...
            tiltGrid, betaGrid, machGrid, alphaGrid, extrapMode);
    end

    objs = cell(1,6);
    for ii = 1:6
        objs{ii} = make_base_coeff_object(fname, sheetLabel, coeffNames{ii}, ...
            F{ii}, tiltGrid, betaGrid, machGrid, alphaGrid, clampToRange);
    end

    cd = objs{1};
    cl = objs{2};
    cm = objs{3};
    cc = objs{4};
    cn = objs{5};
    cll = objs{6};

    lookup_cache(cache_key) = {cd, cl, cm, cc, cn, cll};
end

function [A, sheetLabel] = read_base_aero_rows(fname, sheetName, sheetTiltAngles)
    sheetNames = normalize_sheet_names(sheetName);
    if isempty(sheetNames)
        error('AeroExcel:BadBaseAero', 'At least one base-aero sheet name is required.');
    end

    if isempty(sheetTiltAngles)
        sheetTiltAngles = nan(1, numel(sheetNames));
    else
        sheetTiltAngles = sheetTiltAngles(:).';
        if isscalar(sheetTiltAngles) && numel(sheetNames) > 1
            sheetTiltAngles = repmat(sheetTiltAngles, 1, numel(sheetNames));
        end
        if numel(sheetTiltAngles) ~= numel(sheetNames)
            error('AeroExcel:BadBaseAero', ...
                'base_sheet_tilt_angle_deg must be empty, scalar, or match the number of base sheets.');
        end
    end

    rows = cell(1, numel(sheetNames));
    for ii = 1:numel(sheetNames)
        splitMode = numel(sheetNames) > 1 || isfinite(sheetTiltAngles(ii));
        rows{ii} = read_one_base_aero_sheet(fname, sheetNames{ii}, sheetTiltAngles(ii), splitMode);
    end
    A = vertcat(rows{:});
    sheetLabel = strjoin(sheetNames, ',');
end

function A = read_one_base_aero_sheet(fname, sheetName, sheetTiltAngle, splitMode)
    raw = readmatrix(fname, 'Sheet', sheetName);
    if isempty(raw)
        error('AeroExcel:BadBaseAero', 'Sheet %s is empty.', sheetName);
    end

    raw = raw(:, any(isfinite(raw), 1));
    raw = raw(all(isfinite(raw), 2), :);
    if isempty(raw)
        error('AeroExcel:BadBaseAero', ...
            'Sheet %s has no complete numeric rows.', sheetName);
    end

    if size(raw, 2) >= 10
        A = raw(:, 1:10);
        return;
    end

    if ~splitMode && size(raw, 2) >= 9
        mach = zeros(size(raw, 1), 1);
        A = [raw(:, 1:2), mach, raw(:, 3:9)];
        return;
    end

    if splitMode && size(raw, 2) >= 9
        tilt = sheetTiltAngle;
        if ~isfinite(tilt)
            tilt = parse_tilt_angle_from_sheet_name(sheetName);
        end
        if ~isfinite(tilt)
            error('AeroExcel:BadBaseAero', ...
                ['Sheet %s has 9 numeric columns, so the tilt angle must be ' ...
                 'provided in cfg.data.aero.base_sheet_tilt_angle_deg or encoded in the sheet name.'], ...
                 sheetName);
        end
        A = [repmat(tilt, size(raw, 1), 1), raw(:, 1:9)];
        return;
    end

    if splitMode && size(raw, 2) >= 8
        tilt = sheetTiltAngle;
        if ~isfinite(tilt)
            tilt = parse_tilt_angle_from_sheet_name(sheetName);
        end
        if ~isfinite(tilt)
            error('AeroExcel:BadBaseAero', ...
                ['Sheet %s has 8 numeric columns, so the tilt angle must be ' ...
                 'provided in cfg.data.aero.base_sheet_tilt_angle_deg or encoded in the sheet name.'], ...
                 sheetName);
        end
        mach = zeros(size(raw, 1), 1);
        A = [repmat(tilt, size(raw, 1), 1), raw(:, 1), mach, raw(:, 2:8)];
        return;
    end

    error('AeroExcel:BadBaseAero', ...
        ['Sheet %s must contain either [tilt beta alpha CD CL Cm CC Cn Cl], ' ...
         '[tilt beta Mach alpha CD CL Cm CC Cn Cl], [beta alpha CD CL Cm CC Cn Cl], ' ...
         'or [beta Mach alpha CD CL Cm CC Cn Cl] for split-tilt sheets.'], sheetName);
end

function names = normalize_sheet_names(sheetName)
    if isstring(sheetName)
        names = cellstr(sheetName(:).');
    elseif ischar(sheetName)
        names = {sheetName};
    elseif iscell(sheetName)
        names = cell(size(sheetName(:).'));
        for ii = 1:numel(sheetName)
            names{ii} = char(string(sheetName{ii}));
        end
    else
        error('AeroExcel:BadBaseAero', 'Sheet name must be a char, string, cell array, or string array.');
    end
    names = names(~cellfun(@isempty, names));
end

function tilt = parse_tilt_angle_from_sheet_name(sheetName)
    token = regexp(char(sheetName), '[-+]?\d+(\.\d+)?', 'match', 'once');
    if isempty(token)
        tilt = NaN;
    else
        tilt = str2double(token);
    end
end

function key = make_excel_cache_key(fname, sheetName, extrapMode, clampToRange, sheetTiltAngles)
    info = dir(fname);
    if isempty(info)
        error('AeroExcel:MissingFile', 'Excel aero database not found: %s', fname);
    end
    file_id = fname;
    if isfield(info, 'folder') && ~isempty(info.folder)
        file_id = fullfile(info.folder, info.name);
    end
    sheet_key = strjoin(normalize_sheet_names(sheetName), ',');
    tilt_key = sprintf('%.17g,', sheetTiltAngles(:));
    key = sprintf('%s|%s|%s|%s|%d|%.17g|%d', ...
        file_id, sheet_key, tilt_key, char(extrapMode), logical(clampToRange), ...
        info.datenum, info.bytes);
end

function F = build_base_interpolants_3d(tilt, beta, alpha, coeff, tiltGrid, betaGrid, alphaGrid, extrapMode)
    nT = numel(tiltGrid);
    nB = numel(betaGrid);
    nA = numel(alphaGrid);
    F = cell(1,6);

    for k = 1:6
        grid = nan(nT, nB, nA);
        for it = 1:nT
            for ib = 1:nB
                for ia = 1:nA
                    idx = (tilt == tiltGrid(it)) & ...
                          (beta == betaGrid(ib)) & ...
                          (alpha == alphaGrid(ia));
                    if nnz(idx) ~= 1
                        error('AeroExcel:BadBaseGrid', ...
                            'Grid point (tilt=%g, beta=%g, alpha=%g) has %d entries.', ...
                            tiltGrid(it), betaGrid(ib), alphaGrid(ia), nnz(idx));
                    end
                    grid(it, ib, ia) = coeff(idx, k);
                end
            end
        end
        F{k} = griddedInterpolant({tiltGrid, betaGrid, alphaGrid}, grid, 'linear', extrapMode);
    end
end

function F = build_base_interpolants_4d(tilt, beta, mach, alpha, coeff, tiltGrid, betaGrid, machGrid, alphaGrid, extrapMode)
    nT = numel(tiltGrid);
    nB = numel(betaGrid);
    nM = numel(machGrid);
    nA = numel(alphaGrid);
    F = cell(1,6);

    for k = 1:6
        grid = nan(nT, nB, nM, nA);
        for it = 1:nT
            for ib = 1:nB
                for im = 1:nM
                    for ia = 1:nA
                        idx = (tilt == tiltGrid(it)) & ...
                              (beta == betaGrid(ib)) & ...
                              (mach == machGrid(im)) & ...
                              (alpha == alphaGrid(ia));
                        if nnz(idx) ~= 1
                            error('AeroExcel:BadBaseGrid', ...
                                'Grid point (tilt=%g, beta=%g, Mach=%g, alpha=%g) has %d entries.', ...
                                tiltGrid(it), betaGrid(ib), machGrid(im), alphaGrid(ia), nnz(idx));
                        end
                        grid(it, ib, im, ia) = coeff(idx, k);
                    end
                end
            end
        end
        F{k} = griddedInterpolant({tiltGrid, betaGrid, machGrid, alphaGrid}, grid, 'linear', extrapMode);
    end
end

function obj = make_base_coeff_object(fname, sheetName, coeffName, F, tiltGrid, betaGrid, machGrid, alphaGrid, clampToRange)
    obj = struct();
    obj.type = "excel_base_aero";
    obj.file = fname;
    obj.sheet = sheetName;
    obj.coeffName = coeffName;
    obj.beta_mode = "absolute";
    obj.tiltGrid = tiltGrid;
    obj.betaGrid = betaGrid;
    obj.machGrid = machGrid;
    obj.alphaGrid = alphaGrid;
    obj.F = F;
    obj.eval = @(tilt, alpha, beta, mach) eval_base_coeff(F, tilt, alpha, beta, mach, ...
        tiltGrid, betaGrid, machGrid, alphaGrid, clampToRange);
    obj.CD = @(tilt, alpha) obj.eval(tilt, alpha, 0, machGrid(1));
end

function val = eval_base_coeff(F, tilt, alpha, beta, mach, tiltGrid, betaGrid, machGrid, alphaGrid, clampToRange)
    if nargin < 5 || isempty(mach), mach = machGrid(1); end
    if nargin < 4 || isempty(beta), beta = 0; end
    if clampToRange
        tilt = clamp_value(tilt, tiltGrid(1), tiltGrid(end));
        beta = clamp_value(beta, betaGrid(1), betaGrid(end));
        mach = clamp_value(mach, machGrid(1), machGrid(end));
        alpha = clamp_value(alpha, alphaGrid(1), alphaGrid(end));
    end
    if numel(machGrid) == 1
        val = F(tilt, beta, alpha);
    else
        val = F(tilt, beta, mach, alpha);
    end
end
function y = clamp_value(x, lo, hi)
    y = min(max(x, lo), hi);
end
