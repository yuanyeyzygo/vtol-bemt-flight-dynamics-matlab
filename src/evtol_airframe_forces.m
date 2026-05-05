function forces = evtol_airframe_forces(airframe, state, control, rho)
%EVTOL_AIRFRAME_FORCES Compute non-rotor forces/moments.
%   airframe.model:
%     'lookup_combined' - legacy combined fuselage+wing lookup tables.
%     'component_bem'   - configurable lifting surfaces plus drag-only body.
%
%   forces = [Fx; Fy; Fz; Mx; My; Mz] in body axes.

arguments
    airframe struct
    state struct
    control struct
    rho double
end

switch lower(string(airframe.model))
    case "lookup_combined"
        forces = lookup_combined_forces(airframe, state, control, rho);
    case "component_bem"
        forces = component_bem_forces(airframe, state, control, rho);
    otherwise
        error('airframe.model must be "lookup_combined" or "component_bem".');
end
end

function forces = lookup_combined_forces(airframe, state, control, rho)
uvw = state.velo_body(:);
pqr = state.angular_velocity_body(:);
ref = airframe.lookup.reference_m(:);
U = uvw + cross(pqr, ref);

velocity = norm(U);
if velocity < eps
    forces = zeros(6, 1);
    return;
end

alpha1 = atan2(U(3), U(1));
beta1 = atan2(-U(2), U(1));
alpha = rad2deg(alpha1);
beta = rad2deg(beta1);
tilt_angle = airframe.lookup.tilt_angle_deg;

coeff = airframe.lookup.coeff;
cd_cof = coeff.cd.CD(tilt_angle, alpha);
cl_cof = coeff.cl.CD(tilt_angle, alpha);
cm_cof = coeff.cm.CD(tilt_angle, alpha);
cc_cof = coeff.cc.CD(tilt_angle, alpha);
cn_cof = coeff.cn.CD(tilt_angle, alpha);
cll_cof = coeff.cll.CD(tilt_angle, alpha);

ctrl = airframe.lookup.controls;
elev_vals = ctrl.elevator.eval(control.elevator_deg, alpha);
rudder_vals = ctrl.rudder.eval(control.rudder_deg, alpha);
aileron_vals = ctrl.aileron.eval(control.aileron_deg, alpha);

dcd = elev_vals(1);
dcl = elev_vals(2);
dcm = elev_vals(3);

dcc_ru = rudder_vals(1);
dcn_ru = rudder_vals(2);
dcll_ru = rudder_vals(3);

dcc_ap = aileron_vals(1);
dcn_ap = aileron_vals(2);
dcll_ap = aileron_vals(3);

S = airframe.lookup.area_m2;
C_a = airframe.lookup.chord_m;
b = airframe.lookup.span_m;
q = 0.5 * rho * velocity^2;

l = q * S * (cl_cof + dcl);
d = q * S * (cd_cof + dcd);
m = q * S * C_a * (cm_cof + dcm);
c = q * S * (cc_cof / 5.0 * beta + dcc_ru + dcc_ap);
n = q * S * b * (cn_cof / 5.0 * beta + dcn_ru + dcn_ap);
ll = q * S * b * (cll_cof / 5.0 * beta + dcll_ru + dcll_ap);

Fx_wind = [-d; -c; -l];
wind2body = [cos(alpha1)*cos(beta1), -cos(alpha1)*sin(beta1), -sin(alpha1); ...
             sin(beta1),              cos(beta1),              0; ...
             sin(alpha1)*cos(beta1), -sin(alpha1)*sin(beta1),  cos(alpha1)];

Fx_body = wind2body * Fx_wind;
moment_from_ref = cross(ref, Fx_body);

forces = [Fx_body(1); Fx_body(2); Fx_body(3); ...
          ll + moment_from_ref(1); ...
          m + moment_from_ref(2); ...
          n + moment_from_ref(3)];
end

function forces = component_bem_forces(airframe, state, control, rho)
forces = zeros(6, 1);
components = airframe.components;

for i = 1:numel(components)
    comp = components(i);
    switch lower(string(comp.type))
        case "lifting_surface"
            forces = forces + lifting_surface_forces(comp, state, control, rho);
        case "fuselage_cd"
            forces = forces + fuselage_cd_forces(comp, state, rho);
        otherwise
            error("Unknown airframe component type: %s", comp.type);
    end
end
end

function forces = lifting_surface_forces(comp, state, control, rho)
loc = comp.loc_m(:);
uvw = state.velo_body(:);
pqr = state.angular_velocity_body(:);
U = uvw + cross(pqr, loc);
V = norm(U);
if V < eps
    forces = zeros(6, 1);
    return;
end

alpha1 = atan2(U(3), U(1));
beta1 = atan2(-U(2), U(1));
q = 0.5 * rho * V^2;
S = comp.area_m2;

control_delta = control_delta_deg(comp, control);

switch lower(string(comp.surface_axis))
    case "horizontal"
        alpha_eff_deg = rad2deg(alpha1) + comp.incidence_deg + control_delta;
        CL = comp.a0_per_rad * deg2rad(alpha_eff_deg - comp.alpha0_deg);
        CD = comp.cd0 + comp.drag_k * CL^2;
        L = q * S * CL;
        D = q * S * CD;
        Fx_wind = [-D; 0; -L];

    case "vertical"
        beta_eff_deg = rad2deg(beta1) + comp.incidence_deg + control_delta;
        CY = comp.a0_per_rad * deg2rad(beta_eff_deg - comp.alpha0_deg);
        CD = comp.cd0 + comp.drag_k * CY^2;
        Y = q * S * CY;
        D = q * S * CD;
        Fx_wind = [-D; -Y; 0];

    otherwise
        error('surface_axis must be "horizontal" or "vertical".');
end

wind2body = [cos(alpha1)*cos(beta1), -cos(alpha1)*sin(beta1), -sin(alpha1); ...
             sin(beta1),              cos(beta1),              0; ...
             sin(alpha1)*cos(beta1), -sin(alpha1)*sin(beta1),  cos(alpha1)];
F_body = wind2body * Fx_wind;
M_body = cross(loc, F_body);
forces = [F_body; M_body];
end

function forces = fuselage_cd_forces(comp, state, rho)
loc = comp.loc_m(:);
uvw = state.velo_body(:);
pqr = state.angular_velocity_body(:);
U = uvw + cross(pqr, loc);
V = norm(U);
if V < eps
    forces = zeros(6, 1);
    return;
end

q = 0.5 * rho * V^2;
D = q * comp.area_m2 * comp.cd0;
F_body = -D * U ./ V;
M_body = cross(loc, F_body);
forces = [F_body; M_body];
end

function delta = control_delta_deg(comp, control)
switch lower(string(comp.control))
    case "none"
        raw = 0;
    case "aileron"
        raw = control.aileron_deg;
    case "elevator"
        raw = control.elevator_deg;
    case "rudder"
        raw = control.rudder_deg;
    otherwise
        error("Unknown component control: %s", comp.control);
end
delta = comp.control_sign * raw;
end
