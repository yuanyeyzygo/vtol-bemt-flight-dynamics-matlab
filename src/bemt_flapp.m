function [errors,forces, beta_vals,dbeta_vals,power,ATT]=bemt_flapp(Trim_var, R,Nb, omega,I_beta, velo, k_beta,rotor_profile,theta00,tilt_angle,angular_speed,rotational_direction,acceleration,angular_acc,rotor_bemt_options,Hub_loc)
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
x_nodes = rotor_profile.x_nodes;
r_nodes = rotor_profile.r_nodes;
dr = rotor_profile.dr;
chord_m = rotor_profile.chord_m;
pretwist_rad = rotor_profile.pretwist_rad;
airfoil_id = rotor_profile.airfoil_id;
use_default_airfoil = rotor_profile.airfoil_is_default;
if use_default_airfoil
    default_airfoil = rotor_profile.default_airfoil;
    cl_alpha_per_rad = default_airfoil.cl_alpha_per_rad;
    cl_max = default_airfoil.cl_max;
    cd0 = default_airfoil.cd0;
    cd_alpha2 = default_airfoil.cd_alpha2;
else
    cl_lookup = rotor_profile.cl_lookup;
    cd_lookup = rotor_profile.cd_lookup;
    rad_to_deg = 180 / pi;
end
vi=Trim_var(end);

dpsi  = 2*pi/Steps;
dt    = dpsi / abs(omega);
V_inf= v12(1);
V_infy=v12(2);
V_z=v12(3);
velocity1=[V_inf;V_infy;V_z];

beta_vals = zeros(Steps+1,Nb);
for blade_number = 1:Nb
    beta_vals(:,blade_number)=Trim_var(blade_number);
end
dbeta_vals= zeros(Steps+1,Nb);
for blade_number = 1:Nb
    dbeta_vals(:,blade_number)=Trim_var(Nb+blade_number);
end

Tb_average=0;
Hb_average=0;
Sb_average=0;
Torque_average=0;
M_average=0;
L_average=0;
MMde=zeros(Steps,N_BE);
MMli=zeros(Steps,N_BE);

flap_ctx = struct();
flap_ctx.N_BE = N_BE;
flap_ctx.x_nodes = x_nodes;
flap_ctx.r_nodes = r_nodes;
flap_ctx.dr = dr;
flap_ctx.chord_m = chord_m;
flap_ctx.pretwist_rad = pretwist_rad;
flap_ctx.airfoil_id = airfoil_id;
flap_ctx.use_default_airfoil = use_default_airfoil;
flap_ctx.vi = vi;
flap_ctx.rho = rho;
flap_ctx.theta0 = theta0;
flap_ctx.rotational_direction = rotational_direction;
flap_ctx.velocity1 = velocity1;
flap_ctx.w_disc = w_disc;
flap_ctx.omega = omega;
flap_ctx.I_beta = I_beta;
flap_ctx.k_beta = k_beta;
flap_ctx.ppp = ppp;
flap_ctx.qqq = qqq;
flap_ctx.uuu = uuu;
flap_ctx.vvv = vvv;
flap_ctx.aaw = aaw;
flap_ctx.aap = aap;
flap_ctx.aaq = aaq;
if use_default_airfoil
    flap_ctx.cl_alpha_per_rad = cl_alpha_per_rad;
    flap_ctx.cl_max = cl_max;
    flap_ctx.cd0 = cd0;
    flap_ctx.cd_alpha2 = cd_alpha2;
else
    flap_ctx.cl_lookup = cl_lookup;
    flap_ctx.cd_lookup = cd_lookup;
    flap_ctx.rad_to_deg = rad_to_deg;
end
flap_integrator_id = rotor_flap_integrator_id(rotor_bemt_options);

for k=1:Nb
    for Az1=1:Steps
        Az=rotational_direction*((Az1-1)*dpsi+(k-1)*2*pi/Nb);
        y_now = [beta_vals(Az1,k); dbeta_vals(Az1,k)];
        switch flap_integrator_id
            case 1
                [ydot, loads, alpha_row, drag_row, lift_row] = rotor_flap_rhs_loads_standalone(y_now, Az, flap_ctx);
                beta_dot_next = y_now(2) + ydot(2) * dt;
                y_next = [y_now(1) + beta_dot_next * dt; beta_dot_next];
            case 2
                [k1, loads1, alpha1_row, drag1_row, lift1_row] = rotor_flap_rhs_loads_standalone(y_now, Az, flap_ctx);
                [k2, loads2] = rotor_flap_rhs_loads_standalone(y_now + 0.5*dt*k1, Az + 0.5*rotational_direction*dpsi, flap_ctx);
                [k3, loads3] = rotor_flap_rhs_loads_standalone(y_now + 0.5*dt*k2, Az + 0.5*rotational_direction*dpsi, flap_ctx);
                [k4, loads4] = rotor_flap_rhs_loads_standalone(y_now + dt*k3, Az + rotational_direction*dpsi, flap_ctx);
                y_next = y_now + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
                loads = rotor_loads_weighted_average_standalone(loads1, loads2, loads3, loads4);
                alpha_row = alpha1_row;
                drag_row = drag1_row;
                lift_row = lift1_row;
            otherwise
                error('BEMTFLAP:BadRotorBemtOptions', 'Unknown flap integrator id.');
        end
        if k==1
            ATT(Az1,:) = alpha_row;
            MMde(Az1,:) = drag_row;
            MMli(Az1,:) = lift_row;
        end
        Tb_average=Tb_average+loads.Tb/Steps;
        Hb_average=Hb_average+loads.Hb/Steps;
        Sb_average=Sb_average+loads.Sb/Steps;
        Torque_average=Torque_average+loads.torque/Steps;
        L_average = L_average + loads.L_hub/Steps;
        M_average = M_average + loads.M_hub/Steps;
        beta_vals(Az1+1,k) = y_next(1);
        dbeta_vals(Az1+1,k)= y_next(2);
    end
end

errors=zeros(2*Nb+1,1);
errors(1:Nb)=beta_vals(Steps+1,:).'-Trim_var(1:Nb);
errors(Nb+1:2*Nb)=dbeta_vals(Steps+1,:).'-Trim_var(Nb+1:2*Nb);
errors(end)=(Tb_average+2*rho*pi*R^2*sqrt((V_z+vi)^2+V_inf^2+V_infy^2)*vi)/(rho*pi*R^2*omega^2);
forcess=[Hb_average;Sb_average;Tb_average];
con_forcess=inv_tiltconversion*forcess;
momentss=[(L_average);M_average;(Torque_average)];
con_momentss=inv_tiltconversion*momentss;
aero_moment=cross(Hub_loc,con_forcess');

forces(1)=con_forcess(1);
forces(2)=con_forcess(2);
forces(3)=con_forcess(3);
forces(4)=con_momentss(1)+aero_moment(1);
forces(5)=con_momentss(2)+aero_moment(2);
forces(6)=con_momentss(3)+aero_moment(3);
power=abs(Torque_average)*abs(omega);
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

function [ydot, loads, alpha_row, drag_row, lift_row] = rotor_flap_rhs_loads_standalone(y, Az, ctx)
beta_now = y(1);
beta_dot = y(2);
Tb_new = 0;
Hb_new = 0;
Sb_new = 0;
M_A = 0;
M_torque = 0;
alpha_row = zeros(1, ctx.N_BE);
drag_row = zeros(1, ctx.N_BE);
lift_row = zeros(1, ctx.N_BE);

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
    drag_row(z) = dD_new;
    lift_row(z) = dL_new;

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
M_bl = (3/2)*(ctx.aaw - ctx.uuu*ctx.qqq + ctx.ppp*ctx.vvv);
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

function loads = rotor_loads_weighted_average_standalone(loads1, loads2, loads3, loads4)
names = fieldnames(loads1);
loads = loads1;
for ii = 1:numel(names)
    name = names{ii};
    loads.(name) = (loads1.(name) + 2*loads2.(name) + 2*loads3.(name) + loads4.(name)) / 6;
end
end
