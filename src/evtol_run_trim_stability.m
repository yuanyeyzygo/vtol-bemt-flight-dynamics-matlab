function trim_results = evtol_run_trim_stability(state, vehicle, cfg, model, trim_options)
%EVTOL_RUN_TRIM_STABILITY Clean public trim and linearization workflow.
%   The solver keeps the original output spirit:
%     Mttt : trim table
%     MMA  : stacked A matrices
%     MMB  : stacked B matrices
%     A    : final speed-point state matrix
%     B_all: final speed-point control matrix
%
%   The implementation uses the configured public rotor and airframe models.
%   Lookup data are only required if RUN_ME has selected lookup switches.

if nargin < 5 || isempty(trim_options)
    trim_options = cfg.trim;
end
trim_options = fill_trim_options(trim_options, vehicle);

if isempty(model)
    error("Model input is empty. Run RUN_ME('config_only') before trim.");
end

rotor_count = numel(vehicle.rotors);
blade_count = vehicle.rotors(1).Nb;
rotor_state_len = 2 * blade_count + 1;
trim_vector = initial_trim_vector(trim_options, rotor_count, blade_count);

n_speed = trim_options.n_speed_points;
Mtt = zeros(9, n_speed);
MMA = zeros(n_speed * 9, 8);
MMB = zeros(n_speed * 9, 7);
point = repmat(empty_point_result(), 1, n_speed);

for i_speed = 1:n_speed
    uvw_earth = trim_options.uvw_earth_mps(:);
    uvw_earth(1) = trim_options.speed_start_mps + (i_speed - 1) * trim_options.speed_step_mps;

    [trim_vector, point(i_speed)] = solve_one_speed( ...
        trim_vector, uvw_earth, state, vehicle, trim_options, rotor_state_len);

    row1 = (i_speed - 1) * 9 + 1;
    row2 = i_speed * 9 - 1;
    MMA(row1:row2, :) = point(i_speed).A;
    MMB(row1:row2, :) = point(i_speed).B_all;

    controls = trim_controls(trim_vector, rotor_count, rotor_state_len);
    Mtt(1, i_speed) = uvw_earth(1);
    Mtt(2, i_speed) = trim_options.vehicle_control_rad(1);
    Mtt(3:6, i_speed) = controls(1:4);
    Mtt(7:8, i_speed) = controls(5:6);
    Mtt(9, i_speed) = point(i_speed).power_W;
end

trim_results = struct();
trim_results.Mtt = Mtt;
trim_results.Mttt = Mtt.';
trim_results.MMA = MMA;
trim_results.MMB = MMB;
trim_results.A = point(end).A;
trim_results.B = point(end).B_rotor;
trim_results.B_all = point(end).B_all;
trim_results.Trim_var = trim_vector;
trim_results.derivatives = point(end).derivatives;
trim_results.fixed_surface_derivatives = point(end).fixed_surface_derivatives;
trim_results.eig_A = eig(point(end).A);
trim_results.speed_points = point;
trim_results.state_names = {'u','w','q','theta','v','p','phi','r'};
trim_results.control_names = {'collective','longitudinal','lateral','yaw','elevator','rudder','aileron'};
trim_results.trim_table_columns = {'speed_mps','elevator_rad','collective_deg','longitudinal_deg', ...
    'lateral_deg','yaw_deg','pitch_rad','roll_rad','power_W'};
trim_results.options = trim_options;
end

function [trim_vector, point] = solve_one_speed(trim_vector, uvw_earth, state, vehicle, options, rotor_state_len)
point = empty_point_result();
tol = options.tol;
max_iter = options.trim_max_iter;
damping = options.damping;

for iter = 1:max_iter
    [residual, eval0] = trim_residual(trim_vector, uvw_earth, state, vehicle, options, rotor_state_len);
    residual_norm = norm(residual, 2);
    if residual_norm < tol
        break;
    end

    J = finite_difference_jacobian(@(x) trim_residual_only(x, uvw_earth, state, vehicle, options, rotor_state_len), ...
        trim_vector, residual, options.fd_step);
    dx = solve_linear_step(J, residual);
    if any(~isfinite(dx))
        warning("Trim Newton step produced non-finite values; stopping this speed point.");
        break;
    end

    trim_vector = trim_vector + damping * dx;
end

[residual, eval0] = trim_residual(trim_vector, uvw_earth, state, vehicle, options, rotor_state_len);
[A, B_rotor, B_all, derivatives, fixed_derivatives] = linearize_trim_point( ...
    trim_vector, eval0.velo_body, state, vehicle, options, rotor_state_len);

point.trim_vector = trim_vector;
point.residual = residual;
point.residual_norm = norm(residual, 2);
point.iterations = iter;
point.converged = point.residual_norm < tol;
point.velo_body = eval0.velo_body;
point.total_forces = eval0.total_forces;
point.rotor_forces = eval0.rotor_forces;
point.airframe_forces = eval0.airframe_forces;
point.power_W = eval0.power_W;
point.A = A;
point.B_rotor = B_rotor;
point.B_all = B_all;
point.derivatives = derivatives;
point.fixed_surface_derivatives = fixed_derivatives;

if ~point.converged
    warning("Trim point %.3f m/s did not meet tolerance. Residual norm = %.3e.", uvw_earth(1), point.residual_norm);
end
end

function r = trim_residual_only(trim_vector, uvw_earth, state, vehicle, options, rotor_state_len)
r = trim_residual(trim_vector, uvw_earth, state, vehicle, options, rotor_state_len);
end

function [residual, eval_out] = trim_residual(trim_vector, uvw_earth, state, vehicle, options, rotor_state_len)
controls = trim_controls(trim_vector, numel(vehicle.rotors), rotor_state_len);
pitch = controls(5);
roll = controls(6);
velo_body = earth_to_body(uvw_earth(:), pitch, roll);
pqr = options.pqr_rad_s(:);

eval_out = evaluate_vehicle_body(trim_vector, velo_body, pqr, state, vehicle, options, rotor_state_len);

m = vehicle.mass_kg;
g = vehicle.g;
F = eval_out.total_forces;
body_error = zeros(6, 1);
body_error(1) = (F(1) - m * g * sin(pitch)) / (m * g);
body_error(2) = (F(2) + m * g * sin(roll) * cos(pitch)) / (m * g);
body_error(3) = (F(3) + m * g * cos(roll) * cos(pitch)) / (m * g);
body_error(4) = F(4) / (m * g * 10);
body_error(5) = F(5) / (m * g * 10);
body_error(6) = F(6) / (m * g * 10);

residual = [eval_out.rotor_errors; body_error];
end

function eval_out = evaluate_vehicle_body(trim_vector, velo_body, pqr, state, vehicle, options, rotor_state_len)
rotor_count = numel(vehicle.rotors);
theta_deg = rotor_theta_deg(trim_vector, vehicle, rotor_state_len);
rotor_errors = zeros(rotor_count * rotor_state_len, 1);
rotor_forces = zeros(6, rotor_count);
rotor_power = zeros(1, rotor_count);

for i = 1:rotor_count
    idx = (i - 1) * rotor_state_len + (1:rotor_state_len);
    [err_i, force_i, power_i] = rotor_bemt_periodic( ...
        trim_vector(idx), vehicle.rotors(i), velo_body, pqr, ...
        options.acceleration_body, options.angular_acceleration_body, theta_deg(i));
    rotor_errors(idx) = err_i(:);
    rotor_forces(:, i) = force_i(:);
    rotor_power(i) = power_i;
end

state_eval = state;
state_eval.velo_body = velo_body(:);
state_eval.angular_velocity_body = pqr(:);
control = vehicle_control_struct(options.vehicle_control_rad);
airframe_forces = evtol_airframe_forces(vehicle.airframe, state_eval, control, vehicle.rho);

eval_out = struct();
eval_out.rotor_errors = rotor_errors;
eval_out.rotor_forces = rotor_forces;
eval_out.airframe_forces = airframe_forces;
eval_out.total_forces = sum(rotor_forces, 2) + airframe_forces(:);
eval_out.power_W = sum(rotor_power);
eval_out.velo_body = velo_body(:);
end

function [A, B_rotor, B_all, derivatives, fixed_derivatives] = linearize_trim_point( ...
    trim_vector, velo_body, state, vehicle, options, rotor_state_len)
base_pqr = options.pqr_rad_s(:);
base_eval = evaluate_vehicle_body(trim_vector, velo_body, base_pqr, state, vehicle, options, rotor_state_len);
base_forces = base_eval.total_forces;

derivatives = zeros(10, 6);
step_state = options.stability_fd_step;
step_control_rad = options.control_fd_step_rad;
step_control_deg = rad2deg(step_control_rad);

for n = 1:10
    x = trim_vector;
    v = velo_body(:);
    pqr = base_pqr;
    step = step_state;

    if n <= 3
        v(n) = v(n) + step;
    elseif n <= 6
        pqr(n - 3) = pqr(n - 3) + step;
    else
        control_index = n - 6;
        control_base = numel(vehicle.rotors) * rotor_state_len;
        x(control_base + control_index) = x(control_base + control_index) + step_control_deg;
        step = step_control_rad;
    end

    x = retrim_rotor_states_for_derivative(x, v, pqr, state, vehicle, options, rotor_state_len);
    eval_p = evaluate_vehicle_body(x, v, pqr, state, vehicle, options, rotor_state_len);
    derivatives(n, :) = ((eval_p.total_forces - base_forces) ./ step).';
end

fixed_derivatives = zeros(3, 6);
for n = 1:3
    options_p = options;
    options_p.vehicle_control_rad(n) = options_p.vehicle_control_rad(n) + step_control_rad;
    eval_p = evaluate_vehicle_body(trim_vector, velo_body, base_pqr, state, vehicle, options_p, rotor_state_len);
    fixed_derivatives(n, :) = ((eval_p.total_forces - base_forces) ./ step_control_rad).';
end

[A, B_rotor, B_all] = assemble_state_space(derivatives, fixed_derivatives, trim_vector, velo_body, vehicle, rotor_state_len);
end

function x = retrim_rotor_states_for_derivative(x, velo_body, pqr, state, vehicle, options, rotor_state_len)
if ~options.retrim_rotor_states_for_derivatives || options.stability_max_iter <= 0
    return;
end

rotor_count = numel(vehicle.rotors);
for i = 1:rotor_count
    idx = (i - 1) * rotor_state_len + (1:rotor_state_len);
    for iter = 1:options.stability_max_iter
        err0 = rotor_state_residual(x, idx, i, velo_body, pqr, vehicle, options, rotor_state_len);
        if norm(err0, 2) < options.rotor_state_tol
            break;
        end
        J = finite_difference_rotor_state_jacobian( ...
            @(xi) rotor_state_residual_with_state(x, idx, i, xi, velo_body, pqr, vehicle, options, rotor_state_len), ...
            x(idx), err0, options.fd_step);
        dx = solve_linear_step(J, err0);
        if any(~isfinite(dx))
            break;
        end
        x(idx) = x(idx) + options.rotor_state_damping * dx;
    end
end

if isempty(state)
    error("Invalid state input.");
end
end

function err = rotor_state_residual_with_state(x, idx, rotor_index, rotor_state, velo_body, pqr, vehicle, options, rotor_state_len)
x(idx) = rotor_state(:);
err = rotor_state_residual(x, idx, rotor_index, velo_body, pqr, vehicle, options, rotor_state_len);
end

function err = rotor_state_residual(x, idx, rotor_index, velo_body, pqr, vehicle, options, rotor_state_len)
theta_deg = rotor_theta_deg(x, vehicle, rotor_state_len);
err = rotor_bemt_periodic( ...
    x(idx), vehicle.rotors(rotor_index), velo_body, pqr, ...
    options.acceleration_body, options.angular_acceleration_body, theta_deg(rotor_index));
err = err(:);
end

function J = finite_difference_rotor_state_jacobian(fun, x, r0, fd_step)
n = numel(x);
m = numel(r0);
J = zeros(m, n);
for i = 1:n
    dx = fd_step * max(1, abs(x(i)));
    xp = x;
    xp(i) = xp(i) + dx;
    rp = fun(xp);
    J(:, i) = (rp(:) - r0(:)) / dx;
end
end

function [A, B_rotor, B_all] = assemble_state_space(derivatives, fixed_derivatives, trim_vector, velo_body, vehicle, rotor_state_len)
D = derivatives;
F = fixed_derivatives;

m = vehicle.mass_kg;
I = vehicle.inertia_kgm2;
Ixx = I(1, 1);
Iyy = I(2, 2);
Izz = I(3, 3);
g = vehicle.g;

Xu = D(1,1); Yu = D(1,2); Zu = D(1,3); Lu = D(1,4); Mu = D(1,5); Nu = D(1,6);
Xv = D(2,1); Yv = D(2,2); Zv = D(2,3); Lv = D(2,4); Mv = D(2,5); Nv = D(2,6);
Xw = D(3,1); Yw = D(3,2); Zw = D(3,3); Lw = D(3,4); Mw = D(3,5); Nw = D(3,6);
Xp = D(4,1); Yp = D(4,2); Zp = D(4,3); Lp = D(4,4); Mp = D(4,5); Np = D(4,6);
Xq = D(5,1); Yq = D(5,2); Zq = D(5,3); Lq = D(5,4); Mq = D(5,5); Nq = D(5,6);
Xr = D(6,1); Yr = D(6,2); Zr = D(6,3); Lr = D(6,4); Mr = D(6,5); Nr = D(6,6);

Xcol = D(7,1); Ycol = D(7,2); Zcol = D(7,3); Lcol = D(7,4); Mcol = D(7,5); Ncol = D(7,6);
Xlon = D(8,1); Ylon = D(8,2); Zlon = D(8,3); Llon = D(8,4); Mlon = D(8,5); Nlon = D(8,6);
Xlat = D(9,1); Ylat = D(9,2); Zlat = D(9,3); Llat = D(9,4); Mlat = D(9,5); Nlat = D(9,6);
Xyaw = D(10,1); Yyaw = D(10,2); Zyaw = D(10,3); Lyaw = D(10,4); Myaw = D(10,5); Nyaw = D(10,6);

Xu_ = Xu/m; Xv_ = Xv/m; Xw_ = Xw/m; Xp_ = Xp/m; Xq_ = Xq/m; Xr_ = Xr/m;
Yu_ = Yu/m; Yv_ = Yv/m; Yw_ = Yw/m; Yp_ = Yp/m; Yq_ = Yq/m; Yr_ = Yr/m;
Zu_ = Zu/m; Zv_ = Zv/m; Zw_ = Zw/m; Zp_ = Zp/m; Zq_ = Zq/m; Zr_ = Zr/m;

Lu_ = Lu/Ixx; Lv_ = Lv/Ixx; Lw_ = Lw/Ixx; Lp_ = Lp/Ixx; Lq_ = Lq/Ixx; Lr_ = Lr/Ixx;
Mu_ = Mu/Iyy; Mv_ = Mv/Iyy; Mw_ = Mw/Iyy; Mp_ = Mp/Iyy; Mq_ = Mq/Iyy; Mr_ = Mr/Iyy;
Nu_ = Nu/Izz; Nv_ = Nv/Izz; Nw_ = Nw/Izz; Np_ = Np/Izz; Nq_ = Nq/Izz; Nr_ = Nr/Izz;

Xcol_ = Xcol/m; Xlon_ = Xlon/m; Xlat_ = Xlat/m; Xyaw_ = Xyaw/m;
Ycol_ = Ycol/m; Ylon_ = Ylon/m; Ylat_ = Ylat/m; Yyaw_ = Yyaw/m;
Zcol_ = Zcol/m; Zlon_ = Zlon/m; Zlat_ = Zlat/m; Zyaw_ = Zyaw/m;
Lcol_ = Lcol/Ixx; Llon_ = Llon/Ixx; Llat_ = Llat/Ixx; Lyaw_ = Lyaw/Ixx;
Mcol_ = Mcol/Iyy; Mlon_ = Mlon/Iyy; Mlat_ = Mlat/Iyy; Myaw_ = Myaw/Iyy;
Ncol_ = Ncol/Izz; Nlon_ = Nlon/Izz; Nlat_ = Nlat/Izz; Nyaw_ = Nyaw/Izz;

controls = trim_controls(trim_vector, numel(vehicle.rotors), rotor_state_len);
theta_e = controls(5);
phi_e = controls(6);
Ue = velo_body(1);
Ve = velo_body(2);
We = velo_body(3);

A = [ Xu_, Xw_, Xq_ - We,               -g*cos(theta_e),                 Xv_, Xp_,       0,                         Xr_ + Ve;
      Zu_, Zw_, Zq_ + Ue,               -g*cos(phi_e)*sin(theta_e),      Zv_, Zp_ - Ve,  -g*sin(phi_e)*cos(theta_e), Zr_;
      Mu_, Mw_, Mq_,                    0,                               Mv_, Mp_,       0,                         Mr_;
      0,   0,   cos(phi_e),             0,                               0,   0,         0,                        -sin(phi_e);
      Yu_, Yw_, Yq_,                   -g*sin(phi_e)*sin(theta_e),       Yv_, Yp_ + We,  g*cos(phi_e)*cos(theta_e),  Yr_ - Ue;
      Lu_, Lw_, Lq_,                    0,                               Lv_, Lp_,       0,                         Lr_;
      0,   0,   sin(phi_e)*tan(theta_e),0,                               0,   1,         0,                         cos(phi_e)*tan(theta_e);
      Nu_, Nw_, Nq_,                    0,                               Nv_, Np_,       0,                         Nr_ ];

B_rotor = [ Xcol_, Xlon_, Xlat_, Xyaw_;
            Zcol_, Zlon_, Zlat_, Zyaw_;
            Mcol_, Mlon_, Mlat_, Myaw_;
            0,     0,     0,     0;
            Ycol_, Ylon_, Ylat_, Yyaw_;
            Lcol_, Llon_, Llat_, Lyaw_;
            0,     0,     0,     0;
            Ncol_, Nlon_, Nlat_, Nyaw_ ];

B_fixed = zeros(8, 3);
B_fixed(1, :) = F(:, 1).' / m;
B_fixed(2, :) = F(:, 3).' / m;
B_fixed(3, :) = F(:, 5).' / Iyy;
B_fixed(5, :) = F(:, 2).' / m;
B_fixed(6, :) = F(:, 4).' / Ixx;
B_fixed(8, :) = F(:, 6).' / Izz;

B_all = [B_rotor, B_fixed];
end

function [errors, forces, power] = rotor_bemt_periodic(rotor_state, rotor, velo_body, pqr, acceleration, angular_acc, theta0_deg)
rho = rotor.rho;
R = rotor.R;
Nb = rotor.Nb;
omega_signed = rotor.omega * rotor.rotational_direction;
omega_abs = abs(omega_signed);
I_beta = rotor.I_beta;
k_beta = rotor.k_beta;
root_cutout = rotor.root_cutout;
n_be = rotor.n_be;
n_az = rotor.n_az;
hub_loc = rotor.hub_loc(:);

tilt_rad = deg2rad(rotor.tilt_angle_deg);
tilt_conversion = [sin(tilt_rad), 0, cos(tilt_rad); ...
                   0,             1, 0; ...
                  -cos(tilt_rad), 0, sin(tilt_rad)];
inv_tilt_conversion = inv(tilt_conversion);

vel_disc = tilt_conversion * (velo_body(:) + cross(pqr(:), hub_loc));
w_disc = tilt_conversion * pqr(:);
a_disc = tilt_conversion * acceleration(:);
ang_acc_disc = tilt_conversion * angular_acc(:);

theta0 = deg2rad(theta0_deg);
dr = (R - root_cutout * R) / n_be;
vi = rotor_state(2 * Nb + 1);
dpsi = 2 * pi / n_az;
dt = dpsi / max(omega_abs, eps);

beta_vals = zeros(n_az + 1, Nb);
dbeta_vals = zeros(n_az + 1, Nb);
beta_vals(1, :) = rotor_state(1:Nb).';
dbeta_vals(1, :) = rotor_state(Nb + (1:Nb)).';

Tb_average = 0;
Hb_average = 0;
Sb_average = 0;
Torque_average = 0;
M_average = 0;
L_average = 0;

for k = 1:Nb
    beta_dot = dbeta_vals(1, k);
    for iaz = 1:n_az
        Tb_new = 0;
        Hb_new = 0;
        Sb_new = 0;
        M_aero = 0;
        M_torque = 0;

        az = rotor.rotational_direction * ((iaz - 1) * dpsi + (k - 1) * 2 * pi / Nb);
        azimuth = [-cos(az), -sin(az), 0; ...
                    sin(az), -cos(az), 0; ...
                    0,        0,       1];
        azimuth_inv = inv(azimuth);

        blade_to_disc = [cos(beta_vals(iaz, k)), 0, -sin(beta_vals(iaz, k)); ...
                         0,                      1,  0; ...
                         sin(beta_vals(iaz, k)), 0,  cos(beta_vals(iaz, k))];
        disc_to_blade = inv(blade_to_disc);

        velocity_az = azimuth * vel_disc;
        w_shaft = azimuth * w_disc + [0; 0; omega_signed];
        w_blade = blade_to_disc * w_shaft + [0; dbeta_vals(iaz, k); 0];
        velocity_blade = blade_to_disc * velocity_az;

        for ibe = 1:n_be
            x = root_cutout + ((1 - root_cutout) / n_be / 2) + (ibe - 1) * ((1 - root_cutout) / n_be);
            r = x * R;
            v_elem = velocity_blade + cross(w_blade, [r; 0; 0]);

            theta = theta0 + deg2rad(rotor_pretwist_deg(rotor, x));
            V_n = v_elem(3) - vi * cos(beta_vals(iaz, k));
            V_t = rotor.rotational_direction * v_elem(2);
            phi = atan2(V_n, V_t);
            alpha = theta + phi;
            mach = sqrt(V_n^2 + V_t^2) / 340;

            [CL, CD] = rotor_airfoil_coeff(rotor, x, rad2deg(alpha), mach);
            chord_m = rotor_chord_m(rotor, x);
            qdyn = 0.5 * rho * (V_t^2 + (vi - V_n)^2);
            dL = qdyn * chord_m * dr * CL;
            dD = qdyn * chord_m * dr * CD;

            T_be = -(dL * cos(phi) + dD * sin(phi));
            D_be = -rotor.rotational_direction * dD * cos(phi) + rotor.rotational_direction * dL * sin(phi);
            M_torque = M_torque + D_be * r;

            blade_force = [0; D_be; T_be];
            blade_force = disc_to_blade * blade_force;
            blade_force = azimuth_inv * blade_force;

            Tb_new = Tb_new + blade_force(3);
            Hb_new = Hb_new + blade_force(1);
            Sb_new = Sb_new + blade_force(2);
            M_aero = M_aero + blade_force(3) * r;
        end

        Tb_average = Tb_average + Tb_new / n_az;
        Hb_average = Hb_average + Hb_new / n_az;
        Sb_average = Sb_average + Sb_new / n_az;
        Torque_average = Torque_average + M_torque / n_az;

        p = w_disc(1);
        q = w_disc(2);
        u = vel_disc(1);
        v = vel_disc(2);
        aw = a_disc(3);
        ap = ang_acc_disc(1);
        aq = ang_acc_disc(2);

        M_cf = -omega_signed^2 * I_beta * beta_vals(iaz, k);
        M_spring = -k_beta * beta_vals(iaz, k);
        M_cor = -2 * I_beta * (p * omega_signed * cos(az) - q * omega_signed * sin(az));
        M_body_acc = I_beta * (ap * sin(az) + aq * cos(az));
        M_lin_acc = 1.5 * (aw - u * q + p * v);
        M_average = M_average + M_spring * cos(az) / n_az;
        L_average = L_average + M_spring * sin(az) / n_az;

        beta_ddot = (M_aero + M_cf + M_spring + M_cor + M_body_acc + M_lin_acc) / I_beta;
        beta_dot = beta_dot + beta_ddot * dt;
        dbeta_vals(iaz + 1, k) = beta_dot;
        beta_vals(iaz + 1, k) = beta_vals(iaz, k) + beta_dot * dt;
    end
end

errors = zeros(2 * Nb + 1, 1);
errors(1:Nb) = beta_vals(end, :).' - rotor_state(1:Nb);
errors(Nb + (1:Nb)) = dbeta_vals(end, :).' - rotor_state(Nb + (1:Nb));
momentum_scale = rho * pi * R^2 * rotor.omega^2;
errors(end) = (Tb_average + 2 * rho * pi * R^2 * ...
    sqrt((vel_disc(3) + vi)^2 + vel_disc(1)^2 + vel_disc(2)^2) * vi) / max(momentum_scale, eps);

force_disc = [Hb_average; Sb_average; Tb_average];
force_body = inv_tilt_conversion * force_disc;
moment_disc = [L_average; M_average; Torque_average];
moment_body = inv_tilt_conversion * moment_disc;
aero_moment = cross(hub_loc, force_body);

forces = [force_body; moment_body + aero_moment];
power = abs(Torque_average) * omega_abs;
end

function J = finite_difference_jacobian(fun, x, r0, fd_step)
n = numel(x);
m = numel(r0);
J = zeros(m, n);
for i = 1:n
    dx = fd_step * max(1, abs(x(i)));
    xp = x;
    xp(i) = xp(i) + dx;
    rp = fun(xp);
    J(:, i) = (rp(:) - r0(:)) / dx;
end
end

function dx = solve_linear_step(J, residual)
rhs = -residual(:);
if rcond(J) > 1e-12
    dx = J \ rhs;
else
    dx = pinv(J) * rhs;
end
end

function trim_vector = initial_trim_vector(options, rotor_count, blade_count)
rotor_state_len = 2 * blade_count + 1;
expected_len = rotor_count * rotor_state_len + 6;
if isfield(options, "initial_vector") && numel(options.initial_vector) == expected_len
    trim_vector = options.initial_vector(:);
    return;
end

initial = options.initial;
trim_vector = zeros(expected_len, 1);
beta = expand_to_length(initial.beta_rad, blade_count, "beta_rad");
beta_dot = expand_to_length(initial.beta_dot_rad, blade_count, "beta_dot_rad");
rotor_state = [beta; beta_dot; initial.induced_velocity];
for i = 1:rotor_count
    idx = (i - 1) * rotor_state_len + (1:rotor_state_len);
    trim_vector(idx) = rotor_state;
end
base = rotor_count * rotor_state_len;
trim_vector(base + (1:6)) = [initial.collective_deg; initial.longitudinal_deg; ...
    initial.lateral_deg; initial.yaw_deg; initial.pitch_rad; initial.roll_rad];
end

function theta = rotor_theta_deg(trim_vector, vehicle, rotor_state_len)
rotor_count = numel(vehicle.rotors);
controls = trim_controls(trim_vector, rotor_count, rotor_state_len);
collective = controls(1);
longitudinal = controls(2);
lateral = controls(3);
yaw = controls(4);

if rotor_count == 6
    theta = [collective + lateral; ...
             collective + yaw + longitudinal; ...
             collective - yaw + longitudinal; ...
             collective - lateral; ...
             collective - yaw - longitudinal; ...
             collective + yaw - longitudinal];
    return;
end

positions = reshape([vehicle.rotors.hub_loc], 3, []).';
x_scale = max(max(abs(positions(:, 1))), eps);
y_scale = max(max(abs(positions(:, 2))), eps);
theta = zeros(rotor_count, 1);
for i = 1:rotor_count
    x_gain = -positions(i, 1) / x_scale;
    y_gain = positions(i, 2) / y_scale;
    yaw_gain = vehicle.rotors(i).rotational_direction;
    theta(i) = collective + longitudinal * x_gain + lateral * y_gain + yaw * yaw_gain;
end
end

function controls = trim_controls(trim_vector, rotor_count, rotor_state_len)
base = rotor_count * rotor_state_len;
controls = trim_vector(base + (1:6));
end

function uvw_body = earth_to_body(uvw_earth, theta, phi)
psi = 0;
cphi = cos(phi); sphi = sin(phi);
cth = cos(theta); sth = sin(theta);
cpsi = cos(psi); spsi = sin(psi);
C_e2b = [cth*cpsi,                  cth*spsi,                  -sth; ...
         sphi*sth*cpsi - cphi*spsi, sphi*sth*spsi + cphi*cpsi, sphi*cth; ...
         cphi*sth*cpsi + sphi*spsi, cphi*sth*spsi - sphi*cpsi, cphi*cth];
uvw_body = C_e2b * uvw_earth(:);
end

function control = vehicle_control_struct(vehicle_control_rad)
control = struct();
control.elevator_deg = rad2deg(vehicle_control_rad(1));
control.rudder_deg = rad2deg(vehicle_control_rad(2));
control.aileron_deg = rad2deg(vehicle_control_rad(3));
end

function [CL, CD] = rotor_airfoil_coeff(rotor, x, alpha_deg, mach)
section_id = find(x <= rotor.section_r_end(:), 1, "first");
if isempty(section_id)
    section_id = numel(rotor.section_r_end);
end
if isfield(rotor, "section_airfoil_id")
    section_id = rotor.section_airfoil_id(section_id);
end
section_id = min(section_id, numel(rotor.airfoils));
airfoil = rotor.airfoils(section_id);

switch lower(string(airfoil.type))
    case "linear"
        alpha_rad = deg2rad(alpha_deg - airfoil.alpha0_deg);
        CL = airfoil.a0 * alpha_rad;
        CD = airfoil.cd0 + airfoil.k * alpha_rad.^2;
    case "c81txt"
        CL = airfoil.cl(alpha_deg, mach);
        CD = airfoil.cd(alpha_deg, mach);
    otherwise
        error("Unsupported rotor airfoil type: %s", airfoil.type);
end
end

function chord = rotor_chord_m(rotor, x)
switch lower(string(rotor.chord.type))
    case "linear"
        chord_value = rotor.chord.root_value + (rotor.chord.tip_value - rotor.chord.root_value) * x;
        if strcmpi(rotor.chord.unit, "c_over_R")
            chord = chord_value * rotor.R;
        else
            chord = chord_value;
        end
    case "function"
        chord_value = rotor.chord.fun(x);
        if strcmpi(rotor.chord.unit, "c_over_R")
            chord = chord_value * rotor.R;
        else
            chord = chord_value;
        end
    otherwise
        error("Unsupported rotor chord type: %s", rotor.chord.type);
end
end

function twist = rotor_pretwist_deg(rotor, x)
switch lower(string(rotor.pretwist.type))
    case "linear"
        twist = rotor.pretwist.root_deg + (rotor.pretwist.tip_deg - rotor.pretwist.root_deg) * x;
    case "function"
        twist = rotor.pretwist.fun(x);
    otherwise
        error("Unsupported rotor pretwist type: %s", rotor.pretwist.type);
end
end

function x = expand_to_length(x, n, label)
x = x(:);
if isscalar(x)
    x = repmat(x, n, 1);
end
if numel(x) ~= n
    error("options.trim.initial.%s must be scalar or length %d.", label, n);
end
end

function options = fill_trim_options(options, vehicle)
if ~isfield(options, "n_speed_points"), options.n_speed_points = 1; end
if ~isfield(options, "speed_start_mps"), options.speed_start_mps = 40; end
if ~isfield(options, "speed_step_mps"), options.speed_step_mps = 10; end
if ~isfield(options, "trim_max_iter"), options.trim_max_iter = 25; end
if ~isfield(options, "stability_max_iter"), options.stability_max_iter = 15; end
if ~isfield(options, "tol"), options.tol = 1e-7; end
if ~isfield(options, "fd_step"), options.fd_step = 1e-4; end
if ~isfield(options, "stability_fd_step"), options.stability_fd_step = 1e-2; end
if ~isfield(options, "control_fd_step_rad"), options.control_fd_step_rad = 1e-2; end
if ~isfield(options, "damping"), options.damping = 0.80; end
if ~isfield(options, "retrim_rotor_states_for_derivatives"), options.retrim_rotor_states_for_derivatives = true; end
if ~isfield(options, "rotor_state_tol"), options.rotor_state_tol = 1e-9; end
if ~isfield(options, "rotor_state_damping"), options.rotor_state_damping = 0.80; end
if ~isfield(options, "uvw_earth_mps"), options.uvw_earth_mps = [40; 0; 0]; end
if ~isfield(options, "pqr_rad_s"), options.pqr_rad_s = [0; 0; 0]; end
if ~isfield(options, "acceleration_body"), options.acceleration_body = [0; 0; 0]; end
if ~isfield(options, "angular_acceleration_body"), options.angular_acceleration_body = [0; 0; 0]; end
if ~isfield(options, "vehicle_control_rad"), options.vehicle_control_rad = [0, 0, 0]; end

if ~isfield(options, "initial")
    options.initial = struct();
end
if ~isfield(options.initial, "beta_rad"), options.initial.beta_rad = zeros(vehicle.rotors(1).Nb, 1); end
if ~isfield(options.initial, "beta_dot_rad"), options.initial.beta_dot_rad = zeros(vehicle.rotors(1).Nb, 1); end
if ~isfield(options.initial, "induced_velocity"), options.initial.induced_velocity = 1.0; end
if ~isfield(options.initial, "collective_deg"), options.initial.collective_deg = vehicle.rotors(1).theta0_deg; end
if ~isfield(options.initial, "longitudinal_deg"), options.initial.longitudinal_deg = 0.0; end
if ~isfield(options.initial, "lateral_deg"), options.initial.lateral_deg = 0.0; end
if ~isfield(options.initial, "yaw_deg"), options.initial.yaw_deg = 0.0; end
if ~isfield(options.initial, "pitch_rad"), options.initial.pitch_rad = 0.0; end
if ~isfield(options.initial, "roll_rad"), options.initial.roll_rad = 0.0; end

options.uvw_earth_mps = options.uvw_earth_mps(:);
options.pqr_rad_s = options.pqr_rad_s(:);
options.acceleration_body = options.acceleration_body(:);
options.angular_acceleration_body = options.angular_acceleration_body(:);
options.vehicle_control_rad = options.vehicle_control_rad(:).';
end

function point = empty_point_result()
point = struct();
point.trim_vector = [];
point.residual = [];
point.residual_norm = NaN;
point.iterations = 0;
point.converged = false;
point.velo_body = [];
point.total_forces = zeros(6, 1);
point.rotor_forces = [];
point.airframe_forces = zeros(6, 1);
point.power_W = NaN;
point.A = zeros(8, 8);
point.B_rotor = zeros(8, 4);
point.B_all = zeros(8, 7);
point.derivatives = zeros(10, 6);
point.fixed_surface_derivatives = zeros(3, 6);
end
