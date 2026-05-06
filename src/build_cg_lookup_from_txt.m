function lookup = build_cg_lookup_from_txt(fname, extrapMode)
%BUILD_CG_LOOKUP_FROM_TXT Aircraft CG lookup by common tilt angle.
%   File format:
%       tilt_angle_deg  x_cg_mm  y_cg_mm  z_cg_mm

if nargin < 2 || strlength(string(extrapMode)) == 0
    extrapMode = "linear";
end

if exist(fname, "file") ~= 2
    error("CG lookup file not found: %s", fname);
end

data = readmatrix(fname, "FileType", "text");
if size(data, 2) < 4
    error("%s must contain four numeric columns: tilt_angle_deg x_cg_mm y_cg_mm z_cg_mm.", fname);
end

tilt = data(:, 1);
xyz = data(:, 2:4);
if any(~isfinite(tilt)) || any(~isfinite(xyz(:)))
    error("%s contains non-finite CG lookup values.", fname);
end
if numel(unique(tilt)) ~= numel(tilt)
    error("%s contains duplicate tilt angles.", fname);
end

[tilt, order] = sort(tilt);
xyz = xyz(order, :);

Fx = griddedInterpolant(tilt, xyz(:, 1), "linear", extrapMode);
Fy = griddedInterpolant(tilt, xyz(:, 2), "linear", extrapMode);
Fz = griddedInterpolant(tilt, xyz(:, 3), "linear", extrapMode);

lookup = struct();
lookup.file = fname;
lookup.tilt_angle_deg = tilt;
lookup.xyz_mm = xyz;
lookup.x = @(tilt_angle_deg) Fx(tilt_angle_deg);
lookup.y = @(tilt_angle_deg) Fy(tilt_angle_deg);
lookup.z = @(tilt_angle_deg) Fz(tilt_angle_deg);
lookup.eval = @(tilt_angle_deg) [Fx(tilt_angle_deg), Fy(tilt_angle_deg), Fz(tilt_angle_deg)];
end
