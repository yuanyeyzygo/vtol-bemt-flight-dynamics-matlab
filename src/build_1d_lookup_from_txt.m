function lookup = build_1d_lookup_from_txt(fname, value_name, extrapMode)
%BUILD_1D_LOOKUP_FROM_TXT Build a one-dimensional linear lookup from text.
%   File format:
%       x value
%
%   x is normally r/R for rotor blade geometry tables. The value column keeps
%   its physical units, for example chord in meters or pretwist in degrees.

if nargin < 2 || strlength(string(value_name)) == 0
    value_name = "value";
end
if nargin < 3 || strlength(string(extrapMode)) == 0
    extrapMode = "linear";
end

if exist(fname, "file") ~= 2
    error("Lookup file not found: %s", fname);
end

data = readmatrix(fname, "FileType", "text");
if size(data, 2) < 2
    error("%s must contain at least two numeric columns: x and %s.", fname, value_name);
end

x = data(:, 1);
value = data(:, 2);
if any(~isfinite(x)) || any(~isfinite(value))
    error("%s contains non-finite lookup values.", fname);
end
if numel(unique(x)) ~= numel(x)
    error("%s contains duplicate x values.", fname);
end

[x, order] = sort(x);
value = value(order);

G = griddedInterpolant(x, value, "linear", extrapMode);

lookup = struct();
lookup.file = fname;
lookup.x = x;
lookup.value = value;
lookup.value_name = string(value_name);
lookup.eval = @(xq) G(xq);
end
