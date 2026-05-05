function vehicle = evtol_configure_vehicle_models(vehicle, data_dir)
%EVTOL_CONFIGURE_VEHICLE_MODELS Load rotor and airframe model data.
%   The rotor fields intentionally mirror single-rotor-bemt-matlab:
%     rotor.airfoil_model  = 'linear' or 'c81txt'
%     rotor.chord_model    = 'linear' or 'lookup'
%     rotor.pretwist_model = 'linear' or 'lookup'

arguments
    vehicle struct
    data_dir {mustBeTextScalar}
end

data_dir = char(data_dir);

configured_rotors = repmat(configure_one_rotor(vehicle.rotors(1), data_dir), 1, numel(vehicle.rotors));
for i = 2:numel(vehicle.rotors)
    configured_rotors(i) = configure_one_rotor(vehicle.rotors(i), data_dir);
end
vehicle.rotors = configured_rotors;

vehicle.airframe = configure_airframe(vehicle.airframe, data_dir);
end

function rotor = configure_one_rotor(rotor, data_dir)
switch lower(string(rotor.airfoil_model))
    case "linear"
        rotor.airfoils(1).type = "linear";
        rotor.airfoils(1).a0 = rotor.airfoil_linear.a0;
        rotor.airfoils(1).alpha0_deg = rotor.airfoil_linear.alpha0_deg;
        rotor.airfoils(1).cd0 = rotor.airfoil_linear.cd0;
        rotor.airfoils(1).k = rotor.airfoil_linear.k;

    case "c81txt"
        max_id = max(rotor.section_airfoil_id);
        if numel(rotor.c81_pairs) < max_id
            error("rotor.c81_pairs must contain at least %d file pairs.", max_id);
        end
        tmpl = struct("type", "", "cl", [], "cd", []);
        rotor.airfoils = repmat(tmpl, 1, max_id);
        for k = 1:max_id
            cl_file = fullfile(data_dir, rotor.c81_pairs(k).cl_file);
            cd_file = fullfile(data_dir, rotor.c81_pairs(k).cd_file);
            require_file(cl_file, "rotor C81 CL");
            require_file(cd_file, "rotor C81 CD");
            rotor.airfoils(k).type = "c81txt";
            rotor.airfoils(k).cl = build_c81_lookup_from_txt_local(cl_file);
            rotor.airfoils(k).cd = build_c81_lookup_from_txt_local(cd_file);
        end

    otherwise
        error('rotor.airfoil_model must be "linear" or "c81txt".');
end

switch lower(string(rotor.chord_model))
    case "linear"
        rotor.chord.type = "linear";
        rotor.chord.unit = rotor.chord_linear.unit;
        rotor.chord.root_value = rotor.chord_linear.root_value;
        rotor.chord.tip_value = rotor.chord_linear.tip_value;

    case "lookup"
        matfile = fullfile(data_dir, rotor.chord_lookup.mat);
        require_file(matfile, "rotor chord lookup");
        S = load(matfile);
        rotor.chord.type = "function";
        rotor.chord.unit = rotor.chord_lookup.unit;
        rotor.chord.fun = make_callable_1arg(S, rotor.chord_lookup.var, matfile);

    otherwise
        error('rotor.chord_model must be "linear" or "lookup".');
end

switch lower(string(rotor.pretwist_model))
    case "linear"
        rotor.pretwist.type = "linear";
        rotor.pretwist.root_deg = rotor.pretwist_linear.root_deg;
        rotor.pretwist.tip_deg = rotor.pretwist_linear.tip_deg;

    case "lookup"
        matfile = fullfile(data_dir, rotor.pretwist_lookup.mat);
        require_file(matfile, "rotor pretwist lookup");
        S = load(matfile);
        rotor.pretwist.type = "function";
        rotor.pretwist.fun = make_callable_1arg(S, rotor.pretwist_lookup.var, matfile);

    otherwise
        error('rotor.pretwist_model must be "linear" or "lookup".');
end
end

function airframe = configure_airframe(airframe, data_dir)
switch lower(string(airframe.model))
    case "lookup_combined"
        names = fieldnames(airframe.lookup.files);
        for i = 1:numel(names)
            name = names{i};
            file = fullfile(data_dir, airframe.lookup.files.(name));
            require_file(file, "airframe combined lookup");
            airframe.lookup.coeff.(name) = build_C_lookup_from_txt(file, "linear", false);
        end

        control_names = fieldnames(airframe.lookup.control_files);
        for i = 1:numel(control_names)
            name = control_names{i};
            file = fullfile(data_dir, airframe.lookup.control_files.(name));
            require_file(file, "airframe control lookup");
            airframe.lookup.controls.(name) = build_fuselage_elevator_lookup(file, "linear", false);
        end

    case "component_bem"
        if ~isfield(airframe, "components") || isempty(airframe.components)
            error("airframe.components must be defined for component_bem mode.");
        end

    otherwise
        error('airframe.model must be "lookup_combined" or "component_bem".');
end
end

function require_file(path, label)
if exist(path, "file") ~= 2
    error("Selected %s, but file not found: %s", label, path);
end
end

function fun = make_callable_1arg(S, varname, matfile)
varname = char(varname);
if isfield(S, varname)
    fun = raw_to_callable(S.(varname), sprintf('variable "%s" in %s', varname, matfile));
    return;
end

if isfield(S, "x_grid") && isfield(S, "val")
    x = S.x_grid(:);
    val = S.val(:);
    [x, order] = sort(x);
    val = val(order);
    G = griddedInterpolant(x, val, "linear", "linear");
    fun = @(q) G(q);
    return;
end

names = fieldnames(S);
if numel(names) == 1
    fun = raw_to_callable(S.(names{1}), sprintf('single variable "%s" in %s', names{1}, matfile));
    return;
end

error('Variable "%s" is missing in %s.', varname, matfile);
end

function fun = raw_to_callable(raw, desc)
if isa(raw, "function_handle")
    fun = @(x) raw(x);
    return;
end

try
    raw(0.5);
    fun = @(x) raw(x);
    return;
catch
end

if isstruct(raw) && isfield(raw, "fittedmodel")
    fitted = raw.fittedmodel;
    try
        fitted(0.5);
        fun = @(x) fitted(x);
        return;
    catch
    end
end

error("The %s is not callable with one input.", desc);
end

function Ffun = build_c81_lookup_from_txt_local(filename)
fid = fopen(filename, "r");
if fid < 0
    error("Cannot open C81 txt file: %s", filename);
end
cleaner = onCleanup(@() fclose(fid)); %#ok<NASGU>

first_line = fgetl(fid);
mach_grid = sscanf(first_line, "%f").';
if isempty(mach_grid)
    error("Failed to read Mach grid from first line of %s", filename);
end

rows = [];
while ~feof(fid)
    line = fgetl(fid);
    vals = sscanf(line, "%f").';
    if isempty(vals)
        continue;
    end
    if numel(vals) ~= numel(mach_grid) + 1
        error("Invalid row length in %s.", filename);
    end
    rows(end + 1, :) = vals; %#ok<AGROW>
end

alpha_grid = rows(:, 1);
coeff = rows(:, 2:end);
[mach_grid, mach_order] = sort(mach_grid);
coeff = coeff(:, mach_order);
[alpha_grid, alpha_order] = sort(alpha_grid);
coeff = coeff(alpha_order, :);
G = griddedInterpolant({alpha_grid, mach_grid}, coeff, "linear", "linear");
Ffun = @(alpha_deg, mach) G(alpha_deg, mach);
end
