function [errors,forces,beta_vals,dbeta_vals,power,ATT]=bemt_flapp_disk(Trim_var, R,Nb, omega,I_beta, velo, k_beta,rotor_profile,theta00,tilt_angle,angular_speed,rotational_direction,acceleration,angular_acc,rotor_bemt_options,Hub_loc)
%BEMT_FLAPP_DISK One-rev averaged BEMT with 0/1 harmonic disk flap states.
%
% The blade-element force integration is retained. The per-blade flap states
% are replaced by disk states:
%   [beta0 beta1c beta1s dbeta0 dbeta1c dbeta1s vi]

Trim_var = Trim_var(:);
if numel(Trim_var) ~= 7
    error('BEMTFLAP:BadDiskState', ...
        'bemt_flapp_disk expects [beta0 beta1c beta1s dbeta0 dbeta1c dbeta1s vi].');
end

rho = rotor_bemt_options.rho_kg_m3;
N_BE = rotor_profile.n_be;
Steps = rotor_bemt_options.azimuth_steps;
ATT=zeros(Steps,N_BE);

veloo=velo+cross(angular_speed,Hub_loc');
tilt_conversion=[sin(deg2rad(tilt_angle)) 0 cos(deg2rad(tilt_angle));0 1 0;-cos(deg2rad(tilt_angle)) 0 sin(deg2rad(tilt_angle))];
inv_tiltconversion=tilt_conversion';
v12=tilt_conversion*veloo;
w_disc=tilt_conversion*angular_speed;
a_disc=tilt_conversion*acceleration;
ang_ace=tilt_conversion*angular_acc;
omega=omega*rotational_direction;
ppp=w_disc(1);
qqq=w_disc(2);
uuu=v12(1);
vvv=v12(2);
aaw=a_disc(3);
aap=ang_ace(1);
aaq=ang_ace(2);

theta0=deg2rad(theta00);
use_default_airfoil = rotor_profile.airfoil_is_default;
if use_default_airfoil
    default_airfoil = rotor_profile.default_airfoil;
else
    rad_to_deg = 180 / pi;
end
vi=Trim_var(end);

dpsi = 2*pi/Steps;
T_rev = 2*pi / abs(omega);
T_state = T_rev;
response_state_update = false;
if isfield(rotor_bemt_options, 'disk_state_dt_s')
    T_state = rotor_bemt_options.disk_state_dt_s;
    response_state_update = true;
end
V_inf= v12(1);
V_infy=v12(2);
V_z=v12(3);
velocity1=[V_inf;V_infy;V_z];

ctx = struct();
ctx.N_BE = N_BE;
ctx.r_nodes = rotor_profile.r_nodes;
ctx.dr = rotor_profile.dr;
ctx.chord_m = rotor_profile.chord_m;
ctx.pretwist_rad = rotor_profile.pretwist_rad;
ctx.airfoil_id = rotor_profile.airfoil_id;
ctx.use_default_airfoil = use_default_airfoil;
if ~use_default_airfoil
    ctx.cl_lookup = rotor_profile.cl_lookup;
    ctx.cd_lookup = rotor_profile.cd_lookup;
end
ctx.vi = vi;
ctx.rho = rho;
ctx.theta0 = theta0;
ctx.rotational_direction = rotational_direction;
ctx.velocity1 = velocity1;
ctx.w_disc = w_disc;
ctx.omega = omega;
ctx.I_beta = I_beta;
ctx.k_beta = k_beta;
ctx.ppp = ppp;
ctx.qqq = qqq;
ctx.uuu = uuu;
ctx.vvv = vvv;
ctx.aaw = aaw;
ctx.aap = aap;
ctx.aaq = aaq;
if use_default_airfoil
    ctx.cl_alpha_per_rad = default_airfoil.cl_alpha_per_rad;
    ctx.cl_max = default_airfoil.cl_max;
    ctx.cd0 = default_airfoil.cd0;
    ctx.cd_alpha2 = default_airfoil.cd_alpha2;
else
    ctx.rad_to_deg = rad_to_deg;
end

q = Trim_var(1:3);
qdot = Trim_var(4:6);

[qdd, loads, ATT] = disk_projected_rhs_and_loads(q, qdot, Nb, Steps, dpsi, ctx, ATT);

flap_integrator_id = rotor_flap_integrator_id(rotor_bemt_options);
if response_state_update
    flap_integrator_id = 2;
    if isfield(rotor_bemt_options, 'disk_state_integrator_id')
        flap_integrator_id = rotor_bemt_options.disk_state_integrator_id;
    elseif isfield(rotor_bemt_options, 'disk_state_integrator')
        flap_integrator_id = rotor_integrator_name_to_id(rotor_bemt_options.disk_state_integrator);
    end
end
switch flap_integrator_id
    case 1
        qdot_next = qdot + qdd * T_state;
        q_next = q + qdot_next * T_state;
    case 2
        if T_state == 0
            q_next = q;
            qdot_next = qdot;
        else
            n_sub = 1;
            if isfield(rotor_bemt_options, 'disk_state_substeps')
                n_sub = max(1, round(rotor_bemt_options.disk_state_substeps));
            end
            h = T_state / n_sub;
            y = [q; qdot];
            for sub = 1:n_sub
                k1 = disk_state_derivative(y, Nb, Steps, dpsi, ctx);
                k2 = disk_state_derivative(y + 0.5*h*k1, Nb, Steps, dpsi, ctx);
                k3 = disk_state_derivative(y + 0.5*h*k2, Nb, Steps, dpsi, ctx);
                k4 = disk_state_derivative(y + h*k3, Nb, Steps, dpsi, ctx);
                y = y + (h/6) * (k1 + 2*k2 + 2*k3 + k4);
            end
            q_next = y(1:3);
            qdot_next = y(4:6);
        end
    otherwise
        error('BEMTFLAP:BadRotorBemtOptions', 'Unknown flap integrator id.');
end

beta_vals = zeros(Steps+1,3);
dbeta_vals = zeros(Steps+1,3);
beta_vals(1,:) = q(:).';
dbeta_vals(1,:) = qdot(:).';
beta_vals(end,:) = q_next(:).';
dbeta_vals(end,:) = qdot_next(:).';

errors=zeros(7,1);
errors(1:3)=q_next-q;
errors(4:6)=qdot_next-qdot;
errors(end)=(loads.Tb+2*rho*pi*R^2*sqrt((V_z+vi)^2+V_inf^2+V_infy^2)*vi)/(rho*pi*R^2*omega^2);

forcess=[loads.Hb;loads.Sb;loads.Tb];
con_forcess=inv_tiltconversion*forcess;
momentss=[loads.L_hub; loads.M_hub; loads.torque];
con_momentss=inv_tiltconversion*momentss;
aero_moment=cross(Hub_loc,con_forcess');

forces=zeros(6,1);
forces(1)=con_forcess(1);
forces(2)=con_forcess(2);
forces(3)=con_forcess(3);
forces(4)=con_momentss(1)+aero_moment(1);
forces(5)=con_momentss(2)+aero_moment(2);
forces(6)=con_momentss(3)+aero_moment(3);
power=abs(loads.torque)*abs(omega);
end

function integrator_id = rotor_flap_integrator_id(rotor_bemt_options)
if isfield(rotor_bemt_options, 'flap_integrator_id')
    integrator_id = rotor_bemt_options.flap_integrator_id;
else
    integrator_id = rotor_integrator_name_to_id(rotor_bemt_options.flap_integrator);
end
end

function integrator_id = rotor_integrator_name_to_id(integrator_name)
switch lower(string(integrator_name))
    case "euler"
        integrator_id = 1;
    case "rk4"
        integrator_id = 2;
    otherwise
        error('BEMTFLAP:BadRotorBemtOptions', ...
            'Rotor flap integrator must be "euler" or "rk4".');
end
end

function dydt = disk_state_derivative(y, Nb, Steps, dpsi, ctx)
q = y(1:3);
qdot = y(4:6);
[qdd, ~] = disk_projected_rhs_and_loads(q, qdot, Nb, Steps, dpsi, ctx, []);
dydt = [qdot; qdd];
end

function [qdd, loads_avg, ATT] = disk_projected_rhs_and_loads(q, qdot, Nb, Steps, dpsi, ctx, ATT)
beta2dot_0 = 0;
beta2dot_1c = 0;
beta2dot_1s = 0;

loads_avg = zero_loads();
for Az1=1:Steps
    Az=ctx.rotational_direction*((Az1-1)*dpsi);
    cpsi = cos(Az);
    spsi = sin(Az);
    beta_now = q(1) + q(2)*cpsi + q(3)*spsi;
    beta_dot = qdot(1) + qdot(2)*cpsi + qdot(3)*spsi + ...
        ctx.omega * (-q(2)*spsi + q(3)*cpsi);

    [ydot, loads, alpha_row] = rotor_flap_rhs_loads_disk([beta_now; beta_dot], Az, ctx);
    beta2dot = ydot(2);

    beta2dot_0 = beta2dot_0 + beta2dot / Steps;
    beta2dot_1c = beta2dot_1c + 2 * beta2dot * cpsi / Steps;
    beta2dot_1s = beta2dot_1s + 2 * beta2dot * spsi / Steps;

    loads_avg = add_scaled_loads(loads_avg, loads, Nb / Steps);
    if ~isempty(ATT)
        ATT(Az1,:) = alpha_row;
    end
end

qdd = zeros(3,1);
qdd(1) = beta2dot_0;
qdd(2) = beta2dot_1c - 2*ctx.omega*qdot(3) + ctx.omega^2*q(2);
qdd(3) = beta2dot_1s + 2*ctx.omega*qdot(2) + ctx.omega^2*q(3);
end

function [ydot, loads, alpha_row] = rotor_flap_rhs_loads_disk(y, Az, ctx)
beta_now = y(1);
beta_dot = y(2);
Tb_new = 0;
Hb_new = 0;
Sb_new = 0;
M_A = 0;
M_torque = 0;
alpha_row = zeros(1, ctx.N_BE);

azimuth_new = [-cos(Az) -sin(Az) 0; sin(Az) -cos(Az) 0; 0 0 1];
azimuth_new2 = azimuth_new';
blade_m2 = [cos(beta_now) 0 -sin(beta_now); 0 1 0; sin(beta_now) 0 cos(beta_now)];
blade_m = blade_m2';
velocity2 = azimuth_new * ctx.velocity1;
w_shaft = azimuth_new * ctx.w_disc + [0; 0; ctx.omega];
w_blade = blade_m2 * w_shaft + [0; beta_dot; 0];
velocity = blade_m2 * velocity2;

for z = 1:ctx.N_BE
    r = ctx.r_nodes(z);
    v_blade_elem = velocity + cross(w_blade, [r;0;0]);
    theta = ctx.theta0 + ctx.pretwist_rad(z);
    V_n = v_blade_elem(3) - ctx.vi*cos(beta_now);
    vt = ctx.rotational_direction * v_blade_elem(2);
    phi_new = atan2(V_n, vt);
    alpha_new = theta + phi_new;
    alpha_row(z) = alpha_new;

    if ctx.use_default_airfoil
        C_Lnew = ctx.cl_alpha_per_rad * alpha_new;
        C_Lnew = max(min(C_Lnew, ctx.cl_max), -ctx.cl_max);
        C_Dnew = ctx.cd0 + ctx.cd_alpha2 * alpha_new^2;
    else
        Mach = sqrt(V_n^2 + vt^2) / 340;
        section_id = ctx.airfoil_id(z);
        alpha_deg = alpha_new * ctx.rad_to_deg;
        C_Lnew = ctx.cl_lookup{section_id}(alpha_deg, Mach);
        C_Dnew = ctx.cd_lookup{section_id}(alpha_deg, Mach);
    end

    q_chord_dr = 0.5 * ctx.rho * (vt^2 + (ctx.vi - V_n)^2) * ctx.chord_m(z) * ctx.dr;
    dL_new = q_chord_dr * C_Lnew;
    dD_new = q_chord_dr * C_Dnew;

    T_be = -(dL_new * cos(phi_new) + dD_new * sin(phi_new));
    D_be = -ctx.rotational_direction*dD_new * cos(phi_new) + ctx.rotational_direction*dL_new * sin(phi_new);
    M_torque = M_torque + D_be*r;

    blade_force_b = [0; D_be; T_be];
    M_hinge_b = cross([r;0;0], blade_force_b);
    M_A = M_A + M_hinge_b(2);

    blade_force = blade_m * blade_force_b;
    blade_force = azimuth_new2 * blade_force;
    Tb_new = Tb_new + blade_force(3);
    Hb_new = Hb_new + blade_force(1);
    Sb_new = Sb_new + blade_force(2);
end

M_CF = -ctx.omega^2 * ctx.I_beta * beta_now;
M_R = -ctx.k_beta * beta_now;
M_cor = -2*ctx.I_beta*(ctx.ppp*ctx.omega*cos(Az) - ctx.qqq*ctx.omega*sin(Az));
M_ba = ctx.I_beta*(ctx.aap*sin(Az) + ctx.aaq*cos(Az));
M_bl = 1.5*(ctx.aaw - ctx.uuu*ctx.qqq + ctx.ppp*ctx.vvv);
beta_2dot = (M_A + M_CF + M_R + M_cor + M_ba + M_bl) / ctx.I_beta;

M_R_blade = [0; -ctx.k_beta*beta_now; 0];
M_hub_blade = -M_R_blade;
M_hub_disc = azimuth_new2 * blade_m * M_hub_blade;

ydot = [beta_dot; beta_2dot];
loads = struct();
loads.Tb = Tb_new;
loads.Hb = Hb_new;
loads.Sb = Sb_new;
loads.torque = M_torque;
loads.L_hub = M_hub_disc(1);
loads.M_hub = M_hub_disc(2);
end

function loads = zero_loads()
loads = struct();
loads.Tb = 0;
loads.Hb = 0;
loads.Sb = 0;
loads.torque = 0;
loads.L_hub = 0;
loads.M_hub = 0;
end

function out = add_scaled_loads(out, loads, scale)
out.Tb = out.Tb + scale * loads.Tb;
out.Hb = out.Hb + scale * loads.Hb;
out.Sb = out.Sb + scale * loads.Sb;
out.torque = out.torque + scale * loads.torque;
out.L_hub = out.L_hub + scale * loads.L_hub;
out.M_hub = out.M_hub + scale * loads.M_hub;
end
