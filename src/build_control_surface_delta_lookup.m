function out = build_control_surface_delta_lookup(fname, extrapMode, clampToRange)
%BUILD_CONTROL_SURFACE_DELTA_LOOKUP Lookup for one physical control surface.
%
% File format, no header, 9 columns per row:
%   deflection_deg   Mach   alpha_deg   dCD   dCL   dCm   dCC   dCn   dCl
%
% Output:
%   out.eval(deflection_deg, alpha_deg, Mach) returns
%   [dCD dCL dCm dCC dCn dCl].

    if nargin < 2 || isempty(extrapMode), extrapMode = 'linear'; end
    if nargin < 3 || isempty(clampToRange), clampToRange = false; end

    A = readmatrix(fname);
    if size(A,2) < 9
        error('ControlSurface:BadFile', ...
            'Need 9 columns: [deflection Mach alpha dCD dCL dCm dCC dCn dCl]. File: %s', fname);
    end
    A = A(:,1:9);

    deflection = A(:,1);
    mach = A(:,2);
    alpha = A(:,3);
    coeff = A(:,4:9);

    if any(~isfinite(A(:)))
        error('ControlSurface:BadFile', 'Non-finite values found in %s.', fname);
    end

    deflectionGrid = unique(deflection, 'sorted');
    machGrid = unique(mach, 'sorted');
    alphaGrid = unique(alpha, 'sorted');
    coeffNames = {'dCD', 'dCL', 'dCm', 'dCC', 'dCn', 'dCl'};

    if numel(machGrid) == 1
        F = build_2d_interpolants(deflection, alpha, coeff, ...
            deflectionGrid, alphaGrid, extrapMode);
    else
        F = build_3d_interpolants(deflection, mach, alpha, coeff, ...
            deflectionGrid, machGrid, alphaGrid, extrapMode);
    end

    out = struct();
    out.file = fname;
    out.deflectionGrid = deflectionGrid;
    out.machGrid = machGrid;
    out.alphaGrid = alphaGrid;
    out.coeffNames = coeffNames;
    out.F = F;
    out.eval = @(deflectionQuery, alphaQuery, machQuery) eval_surface_delta( ...
        F, deflectionQuery, alphaQuery, machQuery, deflectionGrid, machGrid, alphaGrid, clampToRange);
    out.eval2 = @(deflectionQuery, alphaQuery) out.eval(deflectionQuery, alphaQuery, machGrid(1));
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
                    error('ControlSurface:BadGrid', ...
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
                        error('ControlSurface:BadGrid', ...
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
