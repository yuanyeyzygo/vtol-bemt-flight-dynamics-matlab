function lookup = build_rotor_position_lookup_from_txt(fname, n_rotors, extrapMode)
%BUILD_ROTOR_POSITION_LOOKUP_FROM_TXT Rotor hub position lookup by tilt angle.
%
% File format, one row per rotor and tilt angle:
%   rotor_id   tilt_angle_deg   x_mm   y_mm   z_mm
%
% The returned positions are absolute aircraft coordinates from the table.
% Conversion to CG-relative body axes is handled by the caller.

if nargin < 2 || isempty(n_rotors)
    n_rotors = [];
end
if nargin < 3 || isempty(extrapMode)
    extrapMode = "linear";
end

A = readmatrix(fname);
assert(size(A, 2) >= 5, ...
    'Need 5 columns: [rotor_id tilt_angle_deg x_mm y_mm z_mm].');

rotor_id = A(:, 1);
tilt = A(:, 2);
xyz = A(:, 3:5);

if isempty(n_rotors)
    n_rotors = max(rotor_id);
end
n_rotors = double(n_rotors);

if any(rotor_id ~= round(rotor_id)) || any(rotor_id < 1) || any(rotor_id > n_rotors)
    error('Rotor ids must be integers from 1 to n_rotors.');
end
if any(~isfinite(A(:)))
    error('Rotor position table contains non-finite values: %s', fname);
end

rotor_ids = (1:n_rotors).';
tilt_grid = unique(tilt, 'sorted');

Fx = cell(n_rotors, 1);
Fy = cell(n_rotors, 1);
Fz = cell(n_rotors, 1);
xyz_grid = nan(n_rotors, numel(tilt_grid), 3);

for i = 1:n_rotors
    rows = rotor_id == rotor_ids(i);
    if nnz(rows) ~= numel(tilt_grid)
        error('Rotor %d must have exactly one row for each tilt angle in %s.', i, fname);
    end

    tilt_i = tilt(rows);
    xyz_i = xyz(rows, :);
    [tilt_i, order] = sort(tilt_i);
    xyz_i = xyz_i(order, :);

    if any(abs(tilt_i - tilt_grid) > 1e-10)
        error('Rotor %d tilt grid does not match the global grid in %s.', i, fname);
    end

    xyz_grid(i, :, :) = xyz_i;
    Fx{i} = griddedInterpolant(tilt_grid, xyz_i(:, 1), 'linear', char(extrapMode));
    Fy{i} = griddedInterpolant(tilt_grid, xyz_i(:, 2), 'linear', char(extrapMode));
    Fz{i} = griddedInterpolant(tilt_grid, xyz_i(:, 3), 'linear', char(extrapMode));
end

lookup = struct();
lookup.file = fname;
lookup.n_rotors = n_rotors;
lookup.tilt_grid_deg = tilt_grid;
lookup.xyz_grid_mm = xyz_grid;
lookup.eval = @(tilt_angle_deg) eval_positions(Fx, Fy, Fz, tilt_angle_deg);
end

function positions_mm = eval_positions(Fx, Fy, Fz, tilt_angle_deg)
n_rotors = numel(Fx);
tilt_angle_deg = expand_tilt_angles(tilt_angle_deg, n_rotors);
positions_mm = zeros(n_rotors, 3);
for i = 1:n_rotors
    tilt_i = tilt_angle_deg(i);
    positions_mm(i, :) = [Fx{i}(tilt_i), Fy{i}(tilt_i), Fz{i}(tilt_i)];
end
end

function tilt_angles = expand_tilt_angles(tilt_angles, n_rotors)
tilt_angles = tilt_angles(:).';
if isscalar(tilt_angles)
    tilt_angles = repmat(tilt_angles, 1, n_rotors);
elseif numel(tilt_angles) ~= n_rotors
    error('tilt_angle_deg must be scalar or contain one value per rotor.');
end
end
