function model = evtol_load_model_data(cfg)
%EVTOL_LOAD_MODEL_DATA Load lookup data or create default model functions.

arguments
    cfg struct
end

model = struct();
model.airfoil = load_airfoil_model(cfg);
model.chord = load_chord_model(cfg);
model.pretwist = load_pretwist_model(cfg);
model.fuselage = load_fuselage_model(cfg);
model.controls = load_control_model(cfg);
model.rotor_positions_m = evtol_rotor_locations(cfg);
end

function airfoil = load_airfoil_model(cfg)
mode = lower(string(cfg.data.airfoil.mode));
nsec = numel(cfg.data.airfoil.section_r_end);
airfoil = struct();
airfoil.mode = mode;
airfoil.section_r_end = cfg.data.airfoil.section_r_end(:).';
airfoil.cl = cell(1, nsec);
airfoil.cd = cell(1, nsec);

switch mode
    case "lookup"
        for k = 1:nsec
            cl_file = data_path(cfg, cfg.data.airfoil.cl_files(k));
            cd_file = data_path(cfg, cfg.data.airfoil.cd_files(k));
            require_file(cl_file, "airfoil CL lookup");
            require_file(cd_file, "airfoil CD lookup");
            airfoil.cl{k} = build_c81_lookup_from_txt(cl_file);
            airfoil.cd{k} = build_c81_lookup_from_txt(cd_file);
        end
    case "default"
        d = cfg.data.airfoil.default;
        for k = 1:nsec
            airfoil.cl{k} = @(alpha_deg, mach) d.a0_per_rad .* ...
                deg2rad(alpha_deg - d.alpha0_deg) + 0 .* mach;
            airfoil.cd{k} = @(alpha_deg, mach) d.cd0 + d.drag_k .* ...
                deg2rad(alpha_deg - d.alpha0_deg).^2 + 0 .* mach;
        end
    otherwise
        error('cfg.data.airfoil.mode must be "lookup" or "default".');
end
end

function chord = load_chord_model(cfg)
mode = lower(string(cfg.data.chord.mode));
chord = struct("mode", mode);

switch mode
    case "lookup"
        matfile = data_path(cfg, cfg.data.chord.lookup_file);
        require_file(matfile, "chord lookup");
        S = load(matfile);
        chord.fun = make_callable_1arg(S, cfg.data.chord.lookup_var, matfile);
    case "default"
        c0 = cfg.data.chord.default_root_c_over_R;
        c1 = cfg.data.chord.default_tip_c_over_R;
        chord.fun = @(x) c0 + (c1 - c0) .* x;
    otherwise
        error('cfg.data.chord.mode must be "lookup" or "default".');
end
end

function pretwist = load_pretwist_model(cfg)
mode = lower(string(cfg.data.pretwist.mode));
pretwist = struct("mode", mode);

switch mode
    case "lookup"
        matfile = data_path(cfg, cfg.data.pretwist.lookup_file);
        require_file(matfile, "pretwist lookup");
        S = load(matfile);
        pretwist.fun = make_callable_1arg(S, cfg.data.pretwist.lookup_var, matfile);
    case "default"
        t0 = cfg.data.pretwist.default_root_deg;
        t1 = cfg.data.pretwist.default_tip_deg;
        pretwist.fun = @(x) t0 + (t1 - t0) .* x;
    otherwise
        error('cfg.data.pretwist.mode must be "lookup" or "default".');
end
end

function fuselage = load_fuselage_model(cfg)
mode = lower(string(cfg.data.fuselage.mode));
fuselage = struct("mode", mode);

switch mode
    case "lookup"
        names = fieldnames(cfg.data.fuselage.files);
        for i = 1:numel(names)
            name = names{i};
            file = data_path(cfg, cfg.data.fuselage.files.(name));
            require_file(file, "fuselage lookup");
            fuselage.(name) = build_C_lookup_from_txt(file, "linear", false);
        end
    case "default"
        d = cfg.data.fuselage.default;
        fuselage.cd = coeff2d(@(tilt, alpha) d.cd0 + d.drag_k .* sind(alpha).^2 + 0 .* tilt);
        fuselage.cl = coeff2d(@(tilt, alpha) d.cl_alpha_per_rad .* deg2rad(alpha) + 0 .* tilt);
        fuselage.cm = coeff2d(@(tilt, alpha) d.cm_alpha_per_rad .* deg2rad(alpha) + 0 .* tilt);
        fuselage.cc = coeff2d(@(tilt, alpha) d.cy_beta_per_rad + 0 .* alpha + 0 .* tilt);
        fuselage.cn = coeff2d(@(tilt, alpha) d.cn_beta_per_rad + 0 .* alpha + 0 .* tilt);
        fuselage.cll = coeff2d(@(tilt, alpha) d.cll_beta_per_rad + 0 .* alpha + 0 .* tilt);
    otherwise
        error('cfg.data.fuselage.mode must be "lookup" or "default".');
end
end

function controls = load_control_model(cfg)
mode = lower(string(cfg.data.controls.mode));
controls = struct("mode", mode);

switch mode
    case "lookup"
        files = cfg.data.controls.files;
        controls.elevator = build_control_lookup(cfg, files.elevator);
        controls.rudder = build_control_lookup(cfg, files.rudder);
        controls.aileron = build_control_lookup(cfg, files.aileron);
    case "default"
        d = cfg.data.controls.default;
        controls.elevator.eval = @(defl, alpha) [0, ...
            d.elevator_dcl_per_rad .* defl, d.elevator_dcm_per_rad .* defl] + 0 .* alpha;
        controls.rudder.eval = @(defl, alpha) [d.rudder_dcy_per_rad .* defl, ...
            d.rudder_dcn_per_rad .* defl, 0] + 0 .* alpha;
        controls.aileron.eval = @(defl, alpha) [0, 0, ...
            d.aileron_dcll_per_rad .* defl] + 0 .* alpha;
    otherwise
        error('cfg.data.controls.mode must be "lookup" or "default".');
end
end

function lookup = build_control_lookup(cfg, file_name)
file = data_path(cfg, file_name);
require_file(file, "control lookup");
lookup = build_fuselage_elevator_lookup(file, "linear", false);
end

function path = data_path(cfg, file_name)
path = fullfile(cfg.paths.data_dir, char(file_name));
end

function require_file(path, label)
if exist(path, "file") ~= 2
    error('Selected %s, but file not found: %s', label, path);
end
end

function fun = make_callable_1arg(S, varname, matfile)
varname = char(varname);
if isfield(S, varname)
    raw = S.(varname);
    fun = make_callable_from_raw(raw, sprintf('variable "%s" in %s', varname, matfile));
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
    raw = S.(names{1});
    fun = make_callable_from_raw(raw, sprintf('single variable "%s" in %s', names{1}, matfile));
    return;
end

error('Variable "%s" is missing in %s, and no supported fallback format was found.', varname, matfile);
end

function fun = make_callable_from_raw(raw, desc)
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

error('The %s is not callable with one input.', desc);
end

function item = coeff2d(fun)
item = struct();
item.CD = fun;
end

function Ffun = build_c81_lookup_from_txt(filename)
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
        error("Invalid C81 row length in %s", filename);
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
