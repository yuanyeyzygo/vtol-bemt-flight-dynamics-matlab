function out = CS1_cd_lookup(alpha_q, mach_q, filename)
%CS1_cd_lookup  2D lookup for CS1 table (alpha, Mach -> Cd) with interpolation.
%
% File format (tab/space delimited):
%   Line 1: Mach numbers (no leading blank cell)
%   Lines 2..end: alpha(deg) then coefficient columns (same count as Machs)
%
% Behaviour:
%   - inside table: bilinear ("linear") interpolation in alpha & Mach
%   - outside table: nearest extrapolation (clamp to edge)
%
% USAGE
%   Cd = CS1_cd_lookup(alpha_q, mach_q);                 % reads "CS1_cd,txt" in current folder
%   Cd = CS1_cd_lookup(alpha_q, mach_q, filename);       % specify file
%   tbl = CS1_cd_lookup([], [], filename);               % returns struct with alpha,mach,Cd,F
%
% Notes:
%   - alpha and mach sample points MUST be unique; duplicates will be removed (first occurrence kept).
%   - alpha/mach will be sorted ascending with Cd re-ordered accordingly.

    if nargin < 3 || isempty(filename)
        filename = "CS1_cd.txt";
    end

    % ---------- Robust parse (handles ragged first row) ----------
    lines = readlines(filename);
    lines = strip(lines);
    lines = lines(lines ~= "");

    if numel(lines) < 2
        error('CS1_cd_lookup:ParseError', 'File appears empty or malformed: %s', filename);
    end

    mach = local_parse_numbers(lines(1)).';
    if isempty(mach)
        error('CS1_cd_lookup:ParseError', 'Could not parse Mach header row.');
    end

    nMach = numel(mach);
    nData = numel(lines) - 1;

    alpha = nan(nData, 1);
    Cd    = nan(nData, nMach);

    for i = 1:nData
        nums = local_parse_numbers(lines(i+1));
        if numel(nums) < 1 + nMach
            error('CS1_cd_lookup:ParseError', ...
                'Line %d has %d numbers; expected at least %d (alpha + %d cols).', ...
                i+1, numel(nums), 1+nMach, nMach);
        end
        alpha(i) = nums(1);
        Cd(i,:)  = nums(2:1+nMach).';
    end

    % ---------- Ensure uniqueness ----------
    [alpha_u, ia] = unique(alpha, 'stable');
    if numel(alpha_u) ~= numel(alpha)
        Cd = Cd(ia,:);
        alpha = alpha_u;
    end

    [mach_u, im] = unique(mach, 'stable');
    if numel(mach_u) ~= numel(mach)
        Cd = Cd(:, im);
        mach = mach_u;
    end

    % ---------- Sort ascending (required by griddedInterpolant) ----------
    [alpha, ia2] = sort(alpha(:), 'ascend');
    Cd = Cd(ia2, :);

    [mach, im2] = sort(mach(:).', 'ascend');
    Cd = Cd(:, im2);

    % ---------- Build interpolant ----------
    F = griddedInterpolant({alpha, mach}, Cd, "linear", "nearest");

    % Table-only output mode
    if nargin >= 1 && isempty(alpha_q) && (nargin < 2 || isempty(mach_q))
        out = struct('alpha', alpha, 'mach', mach, 'Cd', Cd, 'F', F, 'filename', filename);
        return;
    end

    if nargin < 2
        error('CS1_cd_lookup:BadInputs', 'Provide both alpha_q and mach_q (or pass [] for table-only output).');
    end

    out = F(alpha_q, mach_q);
end

function nums = local_parse_numbers(line)
% Parse all numeric tokens in a line (handles tabs/spaces, scientific notation).
    % Replace tabs with spaces for simplicity
    line = replace(line, char(9), ' ');
    toks = regexp(line, '[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', 'match');
    if isempty(toks)
        nums = [];
    else
        nums = str2double(toks);
    end
end
