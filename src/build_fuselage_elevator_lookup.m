function out = build_fuselage_elevator_lookup(fname, extrapMode, clampToRange)
%BUILD_FUSELAGE_ELEVATOR_LOOKUP  2D linear lookup for (de, alpha)->(dCD,dCL,dCM)
%
% File format (5 columns per row):
%   de_deg   alpha_deg   dCD   dCL   dCM
%
% Inputs:
%   fname        : e.g. 'fuselage_elevator.txt'
%   extrapMode   : 'linear' (default) / 'none'  (outside grid)
%   clampToRange : true/false (default false)
%                 - true  : clamp query (de,alpha) to grid bounds (still linear inside)
%                 - false : use extrapMode behavior (linear extrap or NaN)
%
% Output struct out:
%   out.deGrid, out.alphaGrid  : grid vectors
%   out.dCDgrid, out.dCLgrid, out.dCMgrid : matrices (nDe x nAlpha)
%   out.FdCD, out.FdCL, out.FdCM : griddedInterpolant objects
%   out.eval(de,alpha) : returns [dCD,dCL,dCM]
%   out.dCD(de,alpha), out.dCL(de,alpha), out.dCM(de,alpha) : convenience handles

    if nargin < 1 || isempty(fname), fname = 'fuselage_elevator.txt'; end
    if nargin < 2 || isempty(extrapMode), extrapMode = 'linear'; end
    if nargin < 3 || isempty(clampToRange), clampToRange = false; end

    A = readmatrix(fname);
    assert(size(A,2) >= 5, 'Need 5 columns: [de alpha dCD dCL dCM].');

    de    = A(:,1);
    alpha = A(:,2);
    dCD   = A(:,3);
    dCL   = A(:,4);
    dCM   = A(:,5);

    deGrid    = unique(de, 'sorted');
    alphaGrid = unique(alpha, 'sorted');

    nDe = numel(deGrid);
    nA  = numel(alphaGrid);

    dCDgrid = nan(nDe, nA);
    dCLgrid = nan(nDe, nA);
    dCMgrid = nan(nDe, nA);

    % Fill grids
    for i = 1:nDe
        for j = 1:nA
            idx = (de==deGrid(i)) & (alpha==alphaGrid(j));
            if nnz(idx) ~= 1
                error('Grid point (de=%g, alpha=%g) has %d entries (need exactly 1).', ...
                      deGrid(i), alphaGrid(j), nnz(idx));
            end
            dCDgrid(i,j) = dCD(idx);
            dCLgrid(i,j) = dCL(idx);
            dCMgrid(i,j) = dCM(idx);
        end
    end

    % Build interpolants: LINEAR interpolation (not nearest)
    FdCD = griddedInterpolant({deGrid, alphaGrid}, dCDgrid, 'linear', extrapMode);
    FdCL = griddedInterpolant({deGrid, alphaGrid}, dCLgrid, 'linear', extrapMode);
    FdCM = griddedInterpolant({deGrid, alphaGrid}, dCMgrid, 'linear', extrapMode);

    out = struct();
    out.deGrid    = deGrid;
    out.alphaGrid = alphaGrid;
    out.dCDgrid   = dCDgrid;
    out.dCLgrid   = dCLgrid;
    out.dCMgrid   = dCMgrid;
    out.FdCD      = FdCD;
    out.FdCL      = FdCL;
    out.FdCM      = FdCM;

    if clampToRange
        out.eval = @(deq,aq) eval_clamped(FdCD, FdCL, FdCM, deq, aq, deGrid, alphaGrid);
    else
        out.eval = @(deq,aq) [FdCD(deq,aq), FdCL(deq,aq), FdCM(deq,aq)];
    end

    out.dCD = @(deq,aq) out.eval(deq,aq); % returns 1x3; keep convenience below
    out.dCL = @(deq,aq) out.eval(deq,aq);
    out.dCM = @(deq,aq) out.eval(deq,aq);

    % Better single-output convenience handles:
    out.getdCD = @(deq,aq) (clampToRange)*FdCD(clamp(deq,deGrid(1),deGrid(end)), clamp(aq,alphaGrid(1),alphaGrid(end))) + ...
                          (~clampToRange)*FdCD(deq,aq);
    out.getdCL = @(deq,aq) (clampToRange)*FdCL(clamp(deq,deGrid(1),deGrid(end)), clamp(aq,alphaGrid(1),alphaGrid(end))) + ...
                          (~clampToRange)*FdCL(deq,aq);
    out.getdCM = @(deq,aq) (clampToRange)*FdCM(clamp(deq,deGrid(1),deGrid(end)), clamp(aq,alphaGrid(1),alphaGrid(end))) + ...
                          (~clampToRange)*FdCM(deq,aq);

    %fprintf('Loaded %s: grid %d(de) x %d(alpha)\n', fname, nDe, nA);
    %fprintf('Example: de=5, alpha=2 -> [dCD dCL dCM] = [%.6g %.6g %.6g]\n', out.eval(5,2));
end

function y = clamp(x, lo, hi)
    y = min(max(x, lo), hi);
end

function v = eval_clamped(FdCD, FdCL, FdCM, deq, aq, deGrid, alphaGrid)
    deq = clamp(deq, deGrid(1), deGrid(end));
    aq  = clamp(aq,  alphaGrid(1), alphaGrid(end));
    v = [FdCD(deq,aq), FdCL(deq,aq), FdCM(deq,aq)];
end
