function out = build_C_lookup_from_txt(fname, extrapMode, clampToRange)
%BUILD_CD_LOOKUP_FROM_TXT  Fast CD lookup from (tau, alpha) using interpolation
%
% File format: 3 columns per row
%   tau_deg   alpha_deg   CD
%
% Inputs:
%   fname        : e.g. 'cd_table.txt'
%   extrapMode   : 'linear' (default) or 'none'
%                 - 'linear': linear extrapolation outside grid
%                 - 'none'  : return NaN outside grid
%   clampToRange : true/false (default false)
%                 - true: query (tau,alpha) will be clamped to data range
%                 - false: use extrapMode behavior
%
% Output out:
%   out.taus, out.alphas : grid vectors
%   out.CDgrid           : matrix size (numel(taus) x numel(alphas))
%   out.F                : griddedInterpolant
%   out.CD               : function handle, CD = out.CD(tau, alpha)

    if nargin < 1 || isempty(fname), fname = 'cd_table.txt'; end
    if nargin < 2 || isempty(extrapMode), extrapMode = 'linear'; end
    if nargin < 3 || isempty(clampToRange), clampToRange = false; end

    data = readmatrix(fname);
    assert(size(data,2) >= 3, 'File must have at least 3 columns: [tau alpha CD].');

    tau   = data(:,1);
    alpha = data(:,2);
    CD    = data(:,3);

    taus   = unique(tau, 'sorted');
    alphas = unique(alpha, 'sorted');

    % Build grid (taus x alphas)
    CDgrid = nan(numel(taus), numel(alphas));
    for i = 1:numel(taus)
        for j = 1:numel(alphas)
            idx = (tau==taus(i)) & (alpha==alphas(j));
            if nnz(idx) ~= 1
                error('Grid point (tau=%g, alpha=%g) has %d entries (need exactly 1).', ...
                      taus(i), alphas(j), nnz(idx));
            end
            CDgrid(i,j) = CD(idx);
        end
    end

    % Interpolant: interpolation is LINEAR (not nearest)
    % extrapMode:
    %   'linear' -> linear extrapolation
    %   'none'   -> NaN outside
    F = griddedInterpolant({taus, alphas}, CDgrid, 'linear', extrapMode);

    out = struct();
    out.taus   = taus;
    out.alphas = alphas;
    out.CDgrid = CDgrid;
    out.F      = F;

    if clampToRange
        out.CD = @(tq, aq) F(clamp(tq, taus(1), taus(end)), clamp(aq, alphas(1), alphas(end)));
    else
        out.CD = @(tq, aq) F(tq, aq);
    end

    % Demo
    %fprintf('Loaded %s: grid %d (tau) x %d (alpha)\n', fname, numel(taus), numel(alphas));
   % fprintf('Example: CD(tau=45, alpha=2.5) = %.6f\n', out.CD(45, 2.5));
end

function y = clamp(x, lo, hi)
    y = min(max(x, lo), hi);
end
