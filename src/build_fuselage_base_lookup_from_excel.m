function [cd, cl, cm, cc, cn, cll] = build_fuselage_base_lookup_from_excel(fname, sheetName, extrapMode, clampToRange)
%BUILD_FUSELAGE_BASE_LOOKUP_FROM_EXCEL Build base-aero lookups from Excel.
%
% Required sheet columns:
%   tilt_angle_deg beta_deg Mach alpha_deg CD CL Cm CC Cn Cl
%
% The returned coefficient objects evaluate absolute coefficients as a
% function of tilt angle, sideslip beta, Mach, and angle of attack.

    if nargin < 2 || isempty(sheetName), sheetName = 'base_aero'; end
    if nargin < 3 || isempty(extrapMode), extrapMode = 'linear'; end
    if nargin < 4 || isempty(clampToRange), clampToRange = false; end

    persistent lookup_cache
    if isempty(lookup_cache)
        lookup_cache = containers.Map('KeyType', 'char', 'ValueType', 'any');
    end
    cache_key = make_excel_cache_key(fname, sheetName, extrapMode, clampToRange);
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

    A = readmatrix(fname, 'Sheet', sheetName);
    A = A(:,1:min(size(A,2),10));
    A = A(all(isfinite(A),2),:);
    if size(A,2) < 10
        error('AeroExcel:BadBaseAero', ...
            'Sheet %s must contain 10 numeric columns: [tilt beta Mach alpha CD CL Cm CC Cn Cl].', sheetName);
    end

    tilt = A(:,1);
    beta = A(:,2);
    mach = A(:,3);
    alpha = A(:,4);
    coeffNames = {'CD', 'CL', 'Cm', 'CC', 'Cn', 'Cl'};
    coeff = A(:,5:10);

    A = [tilt(:), beta(:), mach(:), alpha(:), coeff];
    if any(~isfinite(A(:)))
        error('AeroExcel:BadBaseAero', 'Non-finite values found in %s sheet %s.', fname, sheetName);
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
        objs{ii} = make_base_coeff_object(fname, sheetName, coeffNames{ii}, ...
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

function key = make_excel_cache_key(fname, sheetName, extrapMode, clampToRange)
    info = dir(fname);
    if isempty(info)
        error('AeroExcel:MissingFile', 'Excel aero database not found: %s', fname);
    end
    file_id = fname;
    if isfield(info, 'folder') && ~isempty(info.folder)
        file_id = fullfile(info.folder, info.name);
    end
    key = sprintf('%s|%s|%s|%d|%.17g|%d', ...
        file_id, char(sheetName), char(extrapMode), logical(clampToRange), ...
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
