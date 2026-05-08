
% BEMTFLAP_SWITCHED
% Main calculation body copied from the original BEMTFLAP program.
% RUN_ME defines cfg/root/data_dir before running this script. The trim,
% BEMT, stability, and control-derivative loops below are intentionally kept
% as close as possible to the original program; the switch layer only changes
% how lookup/default data are supplied.

if ~exist('cfg', 'var') || isempty(cfg)
    cfg = bemtflap_default_cfg();
else
    cfg = bemtflap_complete_cfg(cfg);
end

if ~exist('root', 'var') || isempty(root)
    root = fileparts(fileparts(mfilename('fullpath')));
end

if ~exist('data_dir', 'var') || isempty(data_dir)
    data_dir = fullfile(root, 'data');
end

addpath(fullfile(root, 'src'));

%% Basic settings
R = cfg.rotor.radius_m;
rho = cfg.environment.rho_kg_m3;
Nb = cfg.rotor.blade_count;
omega = cfg.rotor.omega_rad_s;
gross_weight = cfg.vehicle.mass_kg;
g = cfg.environment.gravity_m_s2;
m = gross_weight;
Ixx = cfg.vehicle.inertia_kg_m2(1);
Iyy = cfg.vehicle.inertia_kg_m2(2);
Izz = cfg.vehicle.inertia_kg_m2(3);
dx = R / 10;
I_beta = cfg.rotor.flap_inertia_kg_m2;
k_beta = cfg.rotor.flap_spring_nm_rad;
Veh_con = cfg.initial.fixed_wing_control(:).';
uvw = cfg.initial.uvw_earth_mps(:).';
pqr = cfg.initial.pqr_rad_s(:).';
n_rotors = 6;
tilt_angle = validate_airframe_tilt_angle(cfg.trim.tilt_angle_deg);
rotor_tilt_angles = setup_rotor_tilt_angles(cfg, tilt_angle, n_rotors);
tilt_angle_1 = rotor_tilt_angles(1);
tilt_angle_2 = rotor_tilt_angles(2);
tilt_angle_3 = rotor_tilt_angles(3);
tilt_angle_4 = rotor_tilt_angles(4);
tilt_angle_5 = rotor_tilt_angles(5);
tilt_angle_6 = rotor_tilt_angles(6);

[x_cg, y_cg, z_cg, first_rotor_x, first_rotor_z, rotor_position_lookup] = setup_legacy_geometry(cfg, data_dir);
fuselage_reference_m = setup_fuselage_reference(cfg);
fuselage_geometry = setup_fuselage_geometry(cfg);
rotor_bemt_options = setup_rotor_bemt_options(cfg, rho);
default_rotor_geometry = setup_default_rotor_geometry(cfg);

xcg = x_cg(tilt_angle);  %% aircraft centre of gravity, mm
ycg = y_cg(tilt_angle);
zcg = z_cg(tilt_angle);
XCG = [xcg ycg zcg];
XCG = XCG / 1000;
Loc_fuselage = XCG;

[cd, cl, cm, cc, cn, cll, elev, rudd, airp] = setup_fuselage_model(cfg, data_dir);
[F, pre_twist, cd1, cd2, cd3, cd4, cd5, cd6, cl1, cl2, cl3, cl4, cl5, cl6] = setup_rotor_model(cfg, data_dir);
airfoil_section_edges = validate_airfoil_section_edges(cfg.rotor.airfoil_section_edges);

%beta0 = 0.15;
%beta0_dot = 0.08;
%beta = beta0;
%beta_dot = beta0_dot;
Steps=360;
dt = 2*pi/180.0/Steps;
velo=zeros(3,1);
velo(1)=uvw(1);
velo(2)=uvw(2);
velo(3)=uvw(3);

velo1=velo;   %% Velocity in the earth coordinate
rotor_state_size = 2*Nb + 1;
rotor_state_end = n_rotors * rotor_state_size;
total_trim_vars = rotor_state_end + 6;
rotor_idx = cell(1, n_rotors);
for rotor_number = 1:n_rotors
    rotor_idx{rotor_number} = (rotor_number-1)*rotor_state_size + (1:rotor_state_size);
end
r1 = rotor_idx{1}; r2 = rotor_idx{2}; r3 = rotor_idx{3};
r4 = rotor_idx{4}; r5 = rotor_idx{5}; r6 = rotor_idx{6};
control_idx = rotor_state_end + (1:6);
collective_idx = control_idx(1);
longitudinal_idx = control_idx(2);
lateral_idx = control_idx(3);
yaw_idx = control_idx(4);
pitch_idx = control_idx(5);
roll_idx = control_idx(6);

Jac_big=zeros(total_trim_vars,total_trim_vars);
orignal_diff=zeros(total_trim_vars,1);
new_diff=zeros(total_trim_vars,1);

%%%%Trim variables - adjust for the trim efficiency improvement



Trim_var=zeros(total_trim_vars,1);
rotor_state_default = [zeros(2*Nb,1); 1];
for rotor_number = 1:n_rotors
    Trim_var(rotor_idx{rotor_number},1) = rotor_state_default;
end
Trim_var(collective_idx)=18;    %collective pitch
Trim_var(longitudinal_idx)=-0.002;     %longitudinal
Trim_var(lateral_idx)=0.00;     %lateral
Trim_var(yaw_idx)=0.00;     %yaw_Control
Trim_var(pitch_idx)=-0.01;     %Pitch
Trim_var(roll_idx)=0.00;     %roll
Trim_var = apply_initial_trim(Trim_var, cfg);
Trim_var_start = Trim_var;
Set_pitch=deg2rad(-15);
Jacobian1=zeros(2*Nb+1);
Jacobian2=zeros(2*Nb+1);
Jacobian3=zeros(2*Nb+1);
Jacobian4=zeros(2*Nb+1);
Jacobian5=zeros(2*Nb+1);
Jacobian6=zeros(2*Nb+1);


forces=zeros(6,1);
forces_error=zeros(3,1);
beta_vals = zeros(Steps,Nb);
beta_vals2 = zeros(Steps,Nb);
dbeta_vals = zeros(Steps,Nb);
dbeta_vals2 = zeros(Steps,Nb);
M=zeros(21,9);

angular_velocity=[0;0;0];
acceleration=[0;0;0];
angular_acc=[0;0;0];
Hub_loc=[0;0;0];

rotational_direction = setup_rotational_direction(cfg, n_rotors);
rotational_direction_1=rotational_direction(1);
rotational_direction_2=rotational_direction(2);
rotational_direction_3=rotational_direction(3);
rotational_direction_4=rotational_direction(4);
rotational_direction_5=rotational_direction(5);
rotational_direction_6=rotational_direction(6);

ATT1=zeros(rotor_bemt_options.azimuth_steps, rotor_bemt_options.blade_element_count);
ATT2=zeros(rotor_bemt_options.azimuth_steps, rotor_bemt_options.blade_element_count);
ATT3=zeros(rotor_bemt_options.azimuth_steps, rotor_bemt_options.blade_element_count);
ATT4=zeros(rotor_bemt_options.azimuth_steps, rotor_bemt_options.blade_element_count);
ATT5=zeros(rotor_bemt_options.azimuth_steps, rotor_bemt_options.blade_element_count);
ATT6=zeros(rotor_bemt_options.azimuth_steps, rotor_bemt_options.blade_element_count);
ATT11=zeros(rotor_bemt_options.azimuth_steps, rotor_bemt_options.blade_element_count);
ATT21=zeros(rotor_bemt_options.azimuth_steps, rotor_bemt_options.blade_element_count);
ATT31=zeros(rotor_bemt_options.azimuth_steps, rotor_bemt_options.blade_element_count);
ATT41=zeros(rotor_bemt_options.azimuth_steps, rotor_bemt_options.blade_element_count);
ATT51=zeros(rotor_bemt_options.azimuth_steps, rotor_bemt_options.blade_element_count);


Mtt=zeros(9, numel(cfg.trim.speed_mps));
Trim_var1=Trim_var;

tt1 = numel(cfg.trim.speed_mps); %% How many speed points you want to calculate

MMA=zeros(tt1*9,8);
MMB=zeros(tt1*9,7);

for tt=1:tt1

velo1(1)=cfg.trim.speed_mps(tt);
row1 = (tt-1)*9 + 1;
row2 = tt*9-1;
body_error=zeros(6,1);
body_error1=zeros(6,1);
body_jaco=zeros(6,6);
forcesall=zeros(6,1);
forcesall1=zeros(6,1);
Veh_con = cfg.initial.fixed_wing_control(:).';
angular_velocity = cfg.initial.pqr_rad_s(:);
fixed=zeros(3,6);
if tt > 1 && ~use_previous_trim_solution(cfg)
    Trim_var = Trim_var_start;
end

    for k=1:cfg.trim.max_iterations
        
        [X1_LOC, X2_LOC, X3_LOC, X4_LOC, X5_LOC, X6_LOC, X_CCG] = rotor_locations(rotor_tilt_angles, XCG, first_rotor_x, first_rotor_z, rotor_position_lookup, fuselage_reference_m, default_rotor_geometry);
        velo=earth2body(velo1,Trim_var,Trim_var(pitch_idx));
        velo2=velo;
        [theta0_1,theta0_2,theta0_3,theta0_4,theta0_5,theta0_6]=control_allocation(Trim_var);
        [errors1,forces1, beta_vals1,dbeta_vals1, power1,ATT1]=bemt_flapp(Trim_var(r1,1), R,Nb, omega,I_beta, velo, k_beta,F,pre_twist,cd1,cd2,cd3,cd4,cd5,cd6,cl1,cl2,cl3,cl4,cl5,cl6,airfoil_section_edges,theta0_1,tilt_angle_1,angular_velocity,rotational_direction_1,acceleration,angular_acc,rotor_bemt_options,X1_LOC);
        [errors2,forces2, beta_vals2,dbeta_vals2, power2,ATT2]=bemt_flapp(Trim_var(r2,1), R,Nb, omega,I_beta, velo, k_beta,F,pre_twist,cd1,cd2,cd3,cd4,cd5,cd6,cl1,cl2,cl3,cl4,cl5,cl6,airfoil_section_edges,theta0_2,tilt_angle_2,angular_velocity,rotational_direction_2,acceleration,angular_acc,rotor_bemt_options,X2_LOC);
        [errors3,forces3, beta_vals3,dbeta_vals3, power3,ATT3]=bemt_flapp(Trim_var(r3,1), R,Nb, omega,I_beta, velo, k_beta,F,pre_twist,cd1,cd2,cd3,cd4,cd5,cd6,cl1,cl2,cl3,cl4,cl5,cl6,airfoil_section_edges,theta0_3,tilt_angle_3,angular_velocity,rotational_direction_3,acceleration,angular_acc,rotor_bemt_options,X3_LOC);
        [errors4,forces4, beta_vals4,dbeta_vals4, power4,ATT4]=bemt_flapp(Trim_var(r4,1), R,Nb, omega,I_beta, velo, k_beta,F,pre_twist,cd1,cd2,cd3,cd4,cd5,cd6,cl1,cl2,cl3,cl4,cl5,cl6,airfoil_section_edges,theta0_4,tilt_angle_4,angular_velocity,rotational_direction_4,acceleration,angular_acc,rotor_bemt_options,X4_LOC);
        [errors5,forces5, beta_vals5,dbeta_vals5, power5,ATT5]=bemt_flapp(Trim_var(r5,1), R,Nb, omega,I_beta, velo, k_beta,F,pre_twist,cd1,cd2,cd3,cd4,cd5,cd6,cl1,cl2,cl3,cl4,cl5,cl6,airfoil_section_edges,theta0_5,tilt_angle_5,angular_velocity,rotational_direction_5,acceleration,angular_acc,rotor_bemt_options,X5_LOC);
        [errors6,forces6, beta_vals6,dbeta_vals6, power6,ATT6]=bemt_flapp(Trim_var(r6,1), R,Nb, omega,I_beta, velo, k_beta,F,pre_twist,cd1,cd2,cd3,cd4,cd5,cd6,cl1,cl2,cl3,cl4,cl5,cl6,airfoil_section_edges,theta0_6,tilt_angle_6,angular_velocity,rotational_direction_6,acceleration,angular_acc,rotor_bemt_options,X6_LOC);    
        fuse_forces=fuselage_aerodynamics(X_CCG,Veh_con,rho,velo,angular_velocity,tilt_angle, cd,cl, cm,cc,cn,cll,elev,rudd, airp,fuselage_geometry);
        forcesall=forces1+forces2+forces3+forces4+forces5+forces6+fuse_forces';
        force333=forces1+forces2+forces3+forces4+forces5+forces6;
        body_error(1)=(forcesall(1)-gross_weight*g*sin(Trim_var(pitch_idx)))/gross_weight/g;
        body_error(2)=(forcesall(2)+gross_weight*g*sin(Trim_var(roll_idx))*cos(Trim_var(pitch_idx)))/gross_weight/g;
        body_error(3)=(forcesall(3)+gross_weight*g*cos(Trim_var(roll_idx))*cos(Trim_var(pitch_idx)))/gross_weight/g;
        body_error(4)=forcesall(4)/gross_weight/g/10;
        body_error(5)=forcesall(5)/gross_weight/g/10;
        body_error(6)=forcesall(6)/gross_weight/g/10;
        orignal_diff(r1)=errors1(:);
        orignal_diff(r2)=errors2(:);
        orignal_diff(r3)=errors3(:);
        orignal_diff(r4)=errors4(:);
        orignal_diff(r5)=errors5(:);
        orignal_diff(r6)=errors6(:);
        orignal_diff(control_idx)=body_error(:);
        if isfield(cfg.trim, 'skip_newton') && cfg.trim.skip_newton
            break;
        end
        for i=1:total_trim_vars
            delta=0.001;
            Trim_var(i)=Trim_var(i)+delta;
            [X1_LOC, X2_LOC, X3_LOC, X4_LOC, X5_LOC, X6_LOC, X_CCG] = rotor_locations(rotor_tilt_angles, XCG, first_rotor_x, first_rotor_z, rotor_position_lookup, fuselage_reference_m, default_rotor_geometry);
            velo=earth2body(velo1,Trim_var,Trim_var(pitch_idx));
            [theta0_1,theta0_2,theta0_3,theta0_4,theta0_5,theta0_6]=control_allocation(Trim_var);
            [errors11,forces11, ~,dbeta_vals1, power7,ATT11]=bemt_flapp(Trim_var(r1,1), R,Nb, omega,I_beta, velo, k_beta,F,pre_twist,cd1,cd2,cd3,cd4,cd5,cd6,cl1,cl2,cl3,cl4,cl5,cl6,airfoil_section_edges,theta0_1,tilt_angle_1,angular_velocity,rotational_direction_1,acceleration,angular_acc,rotor_bemt_options,X1_LOC);
            [errors21,forces12, ~,dbeta_vals2, power8,ATT21]=bemt_flapp(Trim_var(r2,1), R,Nb, omega,I_beta, velo, k_beta,F,pre_twist,cd1,cd2,cd3,cd4,cd5,cd6,cl1,cl2,cl3,cl4,cl5,cl6,airfoil_section_edges,theta0_2,tilt_angle_2,angular_velocity,rotational_direction_2,acceleration,angular_acc,rotor_bemt_options,X2_LOC);
            [errors31,forces13, ~,dbeta_vals3, power9,ATT31]=bemt_flapp(Trim_var(r3,1), R,Nb, omega,I_beta, velo, k_beta,F,pre_twist,cd1,cd2,cd3,cd4,cd5,cd6,cl1,cl2,cl3,cl4,cl5,cl6,airfoil_section_edges,theta0_3,tilt_angle_3,angular_velocity,rotational_direction_3,acceleration,angular_acc,rotor_bemt_options,X3_LOC);
            [errors41,forces14, ~,dbeta_vals4, power10,ATT41]=bemt_flapp(Trim_var(r4,1), R,Nb, omega,I_beta, velo, k_beta,F,pre_twist,cd1,cd2,cd3,cd4,cd5,cd6,cl1,cl2,cl3,cl4,cl5,cl6,airfoil_section_edges,theta0_4,tilt_angle_4,angular_velocity,rotational_direction_4,acceleration,angular_acc,rotor_bemt_options,X4_LOC);
            [errors51,forces15, ~,dbeta_vals5, power11,ATT51]=bemt_flapp(Trim_var(r5,1), R,Nb, omega,I_beta, velo, k_beta,F,pre_twist,cd1,cd2,cd3,cd4,cd5,cd6,cl1,cl2,cl3,cl4,cl5,cl6,airfoil_section_edges,theta0_5,tilt_angle_5,angular_velocity,rotational_direction_5,acceleration,angular_acc,rotor_bemt_options,X5_LOC);
            [errors61,forces16, ~,dbeta_vals6, power12,ATT61]=bemt_flapp(Trim_var(r6,1), R,Nb, omega,I_beta, velo, k_beta,F,pre_twist,cd1,cd2,cd3,cd4,cd5,cd6,cl1,cl2,cl3,cl4,cl5,cl6,airfoil_section_edges,theta0_6,tilt_angle_6,angular_velocity,rotational_direction_6,acceleration,angular_acc,rotor_bemt_options,X6_LOC);    
            fuse_forces1=fuselage_aerodynamics(X_CCG,Veh_con,rho,velo,angular_velocity, tilt_angle, cd,cl, cm,cc,cn,cll,elev,rudd, airp,fuselage_geometry);
            forcesall1=forces11+forces12+forces13+forces14+forces15+forces16+fuse_forces1';
            body_error1(1)=(forcesall1(1)-gross_weight*g*sin(Trim_var(pitch_idx)))/gross_weight/g;
            body_error1(2)=(forcesall1(2)+gross_weight*g*sin(Trim_var(roll_idx))*cos(Trim_var(pitch_idx)))/gross_weight/g;
            body_error1(3)=(forcesall1(3)+gross_weight*g*cos(Trim_var(roll_idx))*cos(Trim_var(pitch_idx)))/gross_weight/g;
            body_error1(4)=forcesall1(4)/gross_weight/g/10;
            body_error1(5)=forcesall1(5)/gross_weight/g/10;
            body_error1(6)=forcesall1(6)/gross_weight/g/10;
            Trim_var(i)=Trim_var(i)-delta;       
            new_diff(r1)=errors11(:);
            new_diff(r2)=errors21(:);
            new_diff(r3)=errors31(:);
            new_diff(r4)=errors41(:);
            new_diff(r5)=errors51(:);
            new_diff(r6)=errors61(:);
            new_diff(control_idx)=body_error1(:);
            all_different=(new_diff-orignal_diff)/delta;
            Jac_big(:,i)=all_different;
        end
        %%step=Jac_big\orignal_diff(:);
        %%Trim_var(1:total_trim_vars,1)=Trim_var(1:total_trim_vars,1)-0.9*step;
        
        step=newton_step_schur(Jac_big,orignal_diff);
        %step=solve_bordered_jacobian(Jac_big,orignal_diff);
        if tt<2
            mm33=0.5;
        else 
            mm33=0.99;
        end
        
        Trim_var(1:total_trim_vars,1)=Trim_var(1:total_trim_vars,1)+mm33*step;
        Final=sum(orignal_diff(:).^2);
        if cfg.output.verbose
            disp(velo1(1));
            disp(Final);
        end
        if Final<1e-08
            break;
        end
        
        
        
    end

    % Recompute the trim-point forces after the last Newton update. The
    % stability finite differences must subtract forces from the same
    % Trim_var used as the perturbation baseline.
    [X1_LOC, X2_LOC, X3_LOC, X4_LOC, X5_LOC, X6_LOC, X_CCG] = rotor_locations(rotor_tilt_angles, XCG, first_rotor_x, first_rotor_z, rotor_position_lookup, fuselage_reference_m, default_rotor_geometry);
    velo = earth2body(velo1, Trim_var, Trim_var(pitch_idx));
    velo2 = velo;
    [theta0_1, theta0_2, theta0_3, theta0_4, theta0_5, theta0_6] = control_allocation(Trim_var);
    [errors1, forces1, beta_vals1, dbeta_vals1, power1, ATT1] = bemt_flapp(Trim_var(r1,1), R, Nb, omega, I_beta, velo, k_beta, F, pre_twist, cd1, cd2, cd3, cd4, cd5, cd6, cl1, cl2, cl3, cl4, cl5, cl6, airfoil_section_edges, theta0_1, tilt_angle_1, angular_velocity, rotational_direction_1, acceleration, angular_acc, rotor_bemt_options, X1_LOC);
    [errors2, forces2, beta_vals2, dbeta_vals2, power2, ATT2] = bemt_flapp(Trim_var(r2,1), R, Nb, omega, I_beta, velo, k_beta, F, pre_twist, cd1, cd2, cd3, cd4, cd5, cd6, cl1, cl2, cl3, cl4, cl5, cl6, airfoil_section_edges, theta0_2, tilt_angle_2, angular_velocity, rotational_direction_2, acceleration, angular_acc, rotor_bemt_options, X2_LOC);
    [errors3, forces3, beta_vals3, dbeta_vals3, power3, ATT3] = bemt_flapp(Trim_var(r3,1), R, Nb, omega, I_beta, velo, k_beta, F, pre_twist, cd1, cd2, cd3, cd4, cd5, cd6, cl1, cl2, cl3, cl4, cl5, cl6, airfoil_section_edges, theta0_3, tilt_angle_3, angular_velocity, rotational_direction_3, acceleration, angular_acc, rotor_bemt_options, X3_LOC);
    [errors4, forces4, beta_vals4, dbeta_vals4, power4, ATT4] = bemt_flapp(Trim_var(r4,1), R, Nb, omega, I_beta, velo, k_beta, F, pre_twist, cd1, cd2, cd3, cd4, cd5, cd6, cl1, cl2, cl3, cl4, cl5, cl6, airfoil_section_edges, theta0_4, tilt_angle_4, angular_velocity, rotational_direction_4, acceleration, angular_acc, rotor_bemt_options, X4_LOC);
    [errors5, forces5, beta_vals5, dbeta_vals5, power5, ATT5] = bemt_flapp(Trim_var(r5,1), R, Nb, omega, I_beta, velo, k_beta, F, pre_twist, cd1, cd2, cd3, cd4, cd5, cd6, cl1, cl2, cl3, cl4, cl5, cl6, airfoil_section_edges, theta0_5, tilt_angle_5, angular_velocity, rotational_direction_5, acceleration, angular_acc, rotor_bemt_options, X5_LOC);
    [errors6, forces6, beta_vals6, dbeta_vals6, power6, ATT6] = bemt_flapp(Trim_var(r6,1), R, Nb, omega, I_beta, velo, k_beta, F, pre_twist, cd1, cd2, cd3, cd4, cd5, cd6, cl1, cl2, cl3, cl4, cl5, cl6, airfoil_section_edges, theta0_6, tilt_angle_6, angular_velocity, rotational_direction_6, acceleration, angular_acc, rotor_bemt_options, X6_LOC);
    fuse_forces = fuselage_aerodynamics(X_CCG, Veh_con, rho, velo, angular_velocity, tilt_angle, cd, cl, cm, cc, cn, cll, elev, rudd, airp, fuselage_geometry);
    forcesall = forces1 + forces2 + forces3 + forces4 + forces5 + forces6 + fuse_forces';
    force333 = forces1 + forces2 + forces3 + forces4 + forces5 + forces6;

    body_error(1) = (forcesall(1) - gross_weight*g*sin(Trim_var(pitch_idx))) / gross_weight / g;
    body_error(2) = (forcesall(2) + gross_weight*g*sin(Trim_var(roll_idx))*cos(Trim_var(pitch_idx))) / gross_weight / g;
    body_error(3) = (forcesall(3) + gross_weight*g*cos(Trim_var(roll_idx))*cos(Trim_var(pitch_idx))) / gross_weight / g;
    body_error(4) = forcesall(4) / gross_weight / g / 10;
    body_error(5) = forcesall(5) / gross_weight / g / 10;
    body_error(6) = forcesall(6) / gross_weight / g / 10;
    orignal_diff(r1) = errors1(:);
    orignal_diff(r2) = errors2(:);
    orignal_diff(r3) = errors3(:);
    orignal_diff(r4) = errors4(:);
    orignal_diff(r5) = errors5(:);
    orignal_diff(r6) = errors6(:);
    orignal_diff(control_idx) = body_error(:);

    if diagnostic_trim_only(cfg)
        Mtt(1,tt) = velo1(1);
        Mtt(2,tt) = Veh_con(1);
        Mtt(3:6,tt) = Trim_var(control_idx(1:4),1);
        Mtt(7:8,tt) = Trim_var(control_idx(5:6),1);
        Mtt(9,tt) = power1 + power2 + power3 + power4 + power5 + power6;
        Mttt = Mtt(:,1:tt)';

        trim_results = struct();
        trim_results.cfg = cfg;
        trim_results.trim_table = Mttt;
        trim_results.final_trim_var = Trim_var;
        trim_results.forcesall = forcesall;
        trim_results.rotor_forces = [forces1, forces2, forces3, forces4, forces5, forces6];
        trim_results.fuselage_forces = fuse_forces(:);
        trim_results.trim_residual = orignal_diff;
        trim_results.trim_residual_norm = sum(orignal_diff(:).^2);
        trim_results.tilt_angle_deg = tilt_angle;
        trim_results.rotor_tilt_angle_deg = rotor_tilt_angles(:);
        trim_results.speed_mps = cfg.trim.speed_mps(:);
        trim_results.n_speed_points = numel(cfg.trim.speed_mps);
        trim_results.rotor_locations_m = [
            X1_LOC(:).';
            X2_LOC(:).';
            X3_LOC(:).';
            X4_LOC(:).';
            X5_LOC(:).';
            X6_LOC(:).'];
        trim_results.has_nan = any(~isfinite(Trim_var(:))) || any(~isfinite(forcesall(:))) || any(~isfinite(orignal_diff(:)));
        assignin('base', 'trim_results', trim_results);
        return;
    end

    Jac_big2=zeros(rotor_state_end,rotor_state_end);
    inver_jac=zeros(rotor_state_end,rotor_state_end);
    derivatives=zeros(12,6);

    derivatives1=zeros(12,6);
    derivatives2=zeros(12,6);
    derivatives3=zeros(12,6);
    derivatives4=zeros(12,6);
    derivatives5=zeros(12,6);
    derivatives6=zeros(12,6);


    if cfg.output.verbose
        disp(Trim_var(control_idx));
    end

    wake_angle=rad2deg(atan((velo(3)-Trim_var(r2(end))*sin(deg2rad(tilt_angle_2)))/(velo(1)+Trim_var(r2(end))*cos(deg2rad(tilt_angle_2)))));

    Veh_con1=Veh_con;
    trim_solution_var = Trim_var;
    angular_velocity_solution = angular_velocity;
    velo_solution = velo2;
    Veh_con_solution = Veh_con1;
    for N = get_stability_derivative_indices(cfg)

    % outer perturbation step
    stepN = 0.01;

    Trim_var = trim_solution_var;
    angular_velocity = angular_velocity_solution;
    velo = velo_solution;
    Veh_con = Veh_con_solution;
    [Trim_var, angular_velocity, velo, Veh_con] = stability_check_up(N, Trim_var, angular_velocity, velo_solution, Veh_con_solution);

    for kk = 1:cfg.stability.max_iterations

        % current state
        %velo = earth2body(velo1, Trim_var, Trim_var(pitch_idx));
        [theta0_1, theta0_2, theta0_3, theta0_4, theta0_5, theta0_6] = control_allocation(Trim_var);
        [X1_LOC, X2_LOC, X3_LOC, X4_LOC, X5_LOC, X6_LOC, X_CCG] = rotor_locations(rotor_tilt_angles, XCG, first_rotor_x, first_rotor_z, rotor_position_lookup, fuselage_reference_m, default_rotor_geometry);

        % baseline residuals and forces
        [errors12, forces21, beta_vals1, dbeta_vals1, power7,  ATT11] = bemt_flapp(Trim_var(r1,1),   R, Nb, omega, I_beta, velo, k_beta, F, pre_twist, cd1, cd2, cd3, cd4, cd5, cd6, cl1, cl2, cl3, cl4, cl5, cl6, airfoil_section_edges, theta0_1, tilt_angle_1, angular_velocity, rotational_direction_1, acceleration, angular_acc, rotor_bemt_options, X1_LOC);
        [errors22, forces22, beta_vals2, dbeta_vals2, power8,  ATT21] = bemt_flapp(Trim_var(r2,1),  R, Nb, omega, I_beta, velo, k_beta, F, pre_twist, cd1, cd2, cd3, cd4, cd5, cd6, cl1, cl2, cl3, cl4, cl5, cl6, airfoil_section_edges, theta0_2, tilt_angle_2, angular_velocity, rotational_direction_2, acceleration, angular_acc, rotor_bemt_options, X2_LOC);
        [errors32, forces23, beta_vals3, dbeta_vals3, power9,  ATT31] = bemt_flapp(Trim_var(r3,1),  R, Nb, omega, I_beta, velo, k_beta, F, pre_twist, cd1, cd2, cd3, cd4, cd5, cd6, cl1, cl2, cl3, cl4, cl5, cl6, airfoil_section_edges, theta0_3, tilt_angle_3, angular_velocity, rotational_direction_3, acceleration, angular_acc, rotor_bemt_options, X3_LOC);
        [errors42, forces24, beta_vals4, dbeta_vals4, power10, ATT41] = bemt_flapp(Trim_var(r4,1),  R, Nb, omega, I_beta, velo, k_beta, F, pre_twist, cd1, cd2, cd3, cd4, cd5, cd6, cl1, cl2, cl3, cl4, cl5, cl6, airfoil_section_edges, theta0_4, tilt_angle_4, angular_velocity, rotational_direction_4, acceleration, angular_acc, rotor_bemt_options, X4_LOC);
        [errors52, forces25, beta_vals5, dbeta_vals5, power11, ATT51] = bemt_flapp(Trim_var(r5,1),  R, Nb, omega, I_beta, velo, k_beta, F, pre_twist, cd1, cd2, cd3, cd4, cd5, cd6, cl1, cl2, cl3, cl4, cl5, cl6, airfoil_section_edges, theta0_5, tilt_angle_5, angular_velocity, rotational_direction_5, acceleration, angular_acc, rotor_bemt_options, X5_LOC);
        [errors62, forces26, beta_vals6, dbeta_vals6, power12, ATT61] = bemt_flapp(Trim_var(r6,1),  R, Nb, omega, I_beta, velo, k_beta, F, pre_twist, cd1, cd2, cd3, cd4, cd5, cd6, cl1, cl2, cl3, cl4, cl5, cl6, airfoil_section_edges, theta0_6, tilt_angle_6, angular_velocity, rotational_direction_6, acceleration, angular_acc, rotor_bemt_options, X6_LOC);

        fuse_forces1 = fuselage_aerodynamics(X_CCG, Veh_con, rho, velo, angular_velocity,tilt_angle, cd,cl, cm,cc,cn,cll,elev,rudd, airp,fuselage_geometry);
        forcesall11 = forces21 + forces22 + forces23 + forces24 + forces25 + forces26+fuse_forces1';

        % full baseline residual
        residual0 = [errors12(:);
                     errors22(:);
                     errors32(:);
                     errors42(:);
                     errors52(:);
                     errors62(:)];

        resnorm = sum(residual0.^2);
        if cfg.output.verbose
            disp(resnorm);
            disp(N);
        end

        if resnorm < 1e-12
            break;
        end

        delta = 1e-3;
        Jac_big2 = zeros(rotor_state_end,rotor_state_end);
        errors_base = {errors12, errors22, errors32, errors42, errors52, errors62};
        rot_dirs = [rotational_direction_1, rotational_direction_2, rotational_direction_3, rotational_direction_4, rotational_direction_5, rotational_direction_6];
        hub_locs = {X1_LOC, X2_LOC, X3_LOC, X4_LOC, X5_LOC, X6_LOC};

        for rotor_number = 1:n_rotors
            idx = rotor_idx{rotor_number};
            for i = idx
                Trim_var(i) = Trim_var(i) + delta;
                velo_p = velo;%earth2body(velo1, Trim_var, Trim_var(pitch_idx));
                [theta0_1p, theta0_2p, theta0_3p, theta0_4p, theta0_5p, theta0_6p] = control_allocation(Trim_var);
                theta0_p = [theta0_1p, theta0_2p, theta0_3p, theta0_4p, theta0_5p, theta0_6p];

                [errors_p, ~, ~, ~, ~, ~] = bemt_flapp(Trim_var(idx,1), R, Nb, omega, I_beta, velo_p, k_beta, F, pre_twist, cd1, cd2, cd3, cd4, cd5, cd6, cl1, cl2, cl3, cl4, cl5, cl6, airfoil_section_edges, theta0_p(rotor_number), rotor_tilt_angles(rotor_number), angular_velocity, rot_dirs(rotor_number), acceleration, angular_acc, rotor_bemt_options, hub_locs{rotor_number});

                Jac_big2(idx,i) = (errors_p(:) - errors_base{rotor_number}(:)) / delta;
                Trim_var(i) = Trim_var(i) - delta;
            end
        end

        % solve 6 independent rotor_state_size systems
        delta_trim = zeros(rotor_state_end,1);

        for rotor_number = 1:n_rotors
            idx = rotor_idx{rotor_number};
            delta_trim(idx) = Jac_big2(idx,idx) \ residual0(idx);
        end

        % damped Newton update
        Trim_var(1:rotor_state_end,1) = Trim_var(1:rotor_state_end,1) - 0.99 * delta_trim;
    end





    % outer derivative
    der = (forcesall11 - forcesall) / stepN;
    derivatives(N,:) = der(:).';

    der1= (forces21-forces1)/ stepN;
    derivatives1(N,:) = der1(:).';

    der2= (forces22-forces2)/ stepN;
    derivatives2(N,:) = der2(:).';

    der3= (forces23-forces3)/ stepN;
    derivatives3(N,:) = der3(:).';

    der4= (forces24-forces4)/ stepN;
    derivatives4(N,:) = der4(:).';

    der5= (forces25-forces5)/ stepN;
    derivatives5(N,:) = der5(:).';

    der6= (forces26-forces6)/ stepN;
    derivatives6(N,:) = der6(:).';




    % Restore the exact trim point before moving to the next perturbation.
    Trim_var = trim_solution_var;
    angular_velocity = angular_velocity_solution;
    velo = velo_solution;
    Veh_con = Veh_con_solution;
    Der = derivatives;
    Der(7:10,:) = Der(7:10,:) * (180/pi);

    Xu = Der(1,1);  Yu = Der(1,2);  Zu = Der(1,3);  Lu = Der(1,4);  Mu = Der(1,5);  Nu = Der(1,6);
    Xv = Der(2,1);  Yv = Der(2,2);  Zv = Der(2,3);  Lv = Der(2,4);  Mv = Der(2,5);  Nv = Der(2,6);
    Xw = Der(3,1);  Yw = Der(3,2);  Zw = Der(3,3);  Lw = Der(3,4);  Mw = Der(3,5);  Nw = Der(3,6);
    
    Xp = Der(4,1);  Yp = Der(4,2);  Zp = Der(4,3);  Lp = Der(4,4);  Mp = Der(4,5);  Np = Der(4,6);
    Xq = Der(5,1);  Yq = Der(5,2);  Zq = Der(5,3);  Lq = Der(5,4);  Mq = Der(5,5);  Nq = Der(5,6);
    Xr = Der(6,1);  Yr = Der(6,2);  Zr = Der(6,3);  Lr = Der(6,4);  Mr = Der(6,5);  Nr = Der(6,6);
    
    Xcol = Der(7,1); Ycol = Der(7,2); Zcol = Der(7,3); Lcol = Der(7,4); Mcol = Der(7,5); Ncol = Der(7,6);
    Xlon = Der(8,1); Ylon = Der(8,2); Zlon = Der(8,3); Llon = Der(8,4); Mlon = Der(8,5); Nlon = Der(8,6);
    Xlat = Der(9,1); Ylat = Der(9,2); Zlat = Der(9,3); Llat = Der(9,4); Mlat = Der(9,5); Nlat = Der(9,6);
    Xyaw = Der(10,1); Yyaw = Der(10,2); Zyaw = Der(10,3); Lyaw = Der(10,4); Myaw = Der(10,5); Nyaw = Der(10,6);

    
    % force rows
    Xu_ = Xu/m; Xv_ = Xv/m; Xw_ = Xw/m; Xp_ = Xp/m; Xq_ = Xq/m; Xr_ = Xr/m;
    Yu_ = Yu/m; Yv_ = Yv/m; Yw_ = Yw/m; Yp_ = Yp/m; Yq_ = Yq/m; Yr_ = Yr/m;
    Zu_ = Zu/m; Zv_ = Zv/m; Zw_ = Zw/m; Zp_ = Zp/m; Zq_ = Zq/m; Zr_ = Zr/m;
    
    % moment rows
    Lu_ = Lu/Ixx; Lv_ = Lv/Ixx; Lw_ = Lw/Ixx; Lp_ = Lp/Ixx; Lq_ = Lq/Ixx; Lr_ = Lr/Ixx;
    Mu_ = Mu/Iyy; Mv_ = Mv/Iyy; Mw_ = Mw/Iyy; Mp_ = Mp/Iyy; Mq_ = Mq/Iyy; Mr_ = Mr/Iyy;
    Nu_ = Nu/Izz; Nv_ = Nv/Izz; Nw_ = Nw/Izz; Np_ = Np/Izz; Nq_ = Nq/Izz; Nr_ = Nr/Izz;
    
    % control rows
    Xcol_ = Xcol/m; Xlon_ = Xlon/m; Xlat_ = Xlat/m; Xyaw_ = Xyaw/m;
    Ycol_ = Ycol/m; Ylon_ = Ylon/m; Ylat_ = Ylat/m; Yyaw_ = Yyaw/m;
    Zcol_ = Zcol/m; Zlon_ = Zlon/m; Zlat_ = Zlat/m; Zyaw_ = Zyaw/m;
    
    Lcol_ = Lcol/Ixx; Llon_ = Llon/Ixx; Llat_ = Llat/Ixx; Lyaw_ = Lyaw/Ixx;
    Mcol_ = Mcol/Iyy; Mlon_ = Mlon/Iyy; Mlat_ = Mlat/Iyy; Myaw_ = Myaw/Iyy;
    Ncol_ = Ncol/Izz; Nlon_ = Nlon/Izz; Nlat_ = Nlat/Izz; Nyaw_ = Nyaw/Izz;
    % trim values
    Ue = velo2(1);                 % body-axis trim u
    Ve = velo2(2);                 % body-axis trim v
    We = velo2(3);                % body-axis trim w
    phi_e   = Trim_var(roll_idx);   % roll
    theta_e = Trim_var(pitch_idx);   % pitch
    g = 9.81;
    
    % state order: [u, w, q, theta, v, p, phi, r]^T
    % control order: [collective, longitudinal, lateral, yaw, Elevator, Rudder, flapprone]^T


    A = [ Xu_  Xw_  Xq_ - We                  -g*cos(theta_e)                 Xv_  Xp_        0                           Xr_ + Ve
          Zu_  Zw_  Zq_ + Ue                  -g*cos(phi_e)*sin(theta_e)      Zv_  Zp_ - Ve   -g*sin(phi_e)*cos(theta_e) Zr_
          Mu_  Mw_  Mq_                       0                               Mv_  Mp_        0                           Mr_
          0    0    cos(phi_e)                0                               0    0          0                           -sin(phi_e)
          Yu_  Yw_  Yq_                       -g*sin(phi_e)*sin(theta_e)      Yv_  Yp_ + We   g*cos(phi_e)*cos(theta_e)  Yr_ - Ue
          Lu_  Lw_  Lq_                       0                               Lv_  Lp_        0                           Lr_
          0    0    sin(phi_e)*tan(theta_e)   0                               0    1          0                           cos(phi_e)*tan(theta_e)
          Nu_  Nw_  Nq_                       0                               Nv_  Np_        0                           Nr_ ];
    
    B = [ Xcol_  Xlon_  Xlat_  Xyaw_
          Zcol_  Zlon_  Zlat_  Zyaw_
          Mcol_  Mlon_  Mlat_  Myaw_
          0      0      0      0
          Ycol_  Ylon_  Ylat_  Yyaw_
          Lcol_  Llon_  Llat_  Lyaw_
          0      0      0      0
          Ncol_  Nlon_  Nlat_  Nyaw_ ];




    end
    Trim_var = trim_solution_var;
    angular_velocity = angular_velocity_solution;
    velo = velo_solution;
    Veh_con = Veh_con_solution;
    for N = get_control_derivative_indices(cfg)
        Trim_var = trim_solution_var;
        angular_velocity = angular_velocity_solution;
        velo = velo_solution;
        Veh_con = Veh_con_solution;
        [Trim_var, angular_velocity, velo, Veh_con] = stability_check_up(N, Trim_var, angular_velocity, velo_solution, Veh_con_solution);
        fuse_forces12 = fuselage_aerodynamics(X_CCG, Veh_con, rho, velo, angular_velocity,tilt_angle, cd,cl, cm,cc,cn,cll,elev,rudd, airp,fuselage_geometry);
        forcesall3=force333+fuse_forces12';
        dforce=(forcesall3-forcesall)/stepN;
        Trim_var = trim_solution_var;
        angular_velocity = angular_velocity_solution;
        velo = velo_solution;
        Veh_con = Veh_con_solution;
        fixed(N-10,1:6)=dforce(1:6);
    end
    fixed(1:3,1:3)=fixed(1:3,1:3)/m;
    fixed(1:3,4)=fixed(1:3,4)/Ixx;
    fixed(1:3,5)=fixed(1:3,5)/Iyy;
    fixed(1:3,6)=fixed(1:3,6)/Izz;
    B_fixedd=fixed'*57.3;
    B_afixedd=zeros(8,3);
    B_afixedd(1,1:3)=B_fixedd(1,1:3);
    B_afixedd(2,1:3)=B_fixedd(3,1:3);
    B_afixedd(3,1:3)=B_fixedd(5,1:3);
    B_afixedd(5,1:3)=B_fixedd(2,1:3);
    B_afixedd(6,1:3)=B_fixedd(4,1:3);
    B_afixedd(8,1:3)=B_fixedd(6,1:3);
    %B_afixedd(5:6,1:3)=B_fixedd(4:5,1:3);
    %B_afixedd(8,1:3)=B_fixedd(6,1:3);
    B_all=zeros(8,7);
    B_all(1:8,1:4)=B(1:8,1:4);
    B_all(1:8,5:7)=B_afixedd(1:8,1:3);
    eig_A = eig(A);
    if cfg.output.verbose
        disp(eig_A);
    end
   


Trim_var = trim_solution_var;
angular_velocity = angular_velocity_solution;
velo = velo_solution;
Veh_con = Veh_con_solution;

Mtt(1,tt)=velo1(1);
Mtt(2,tt)=Veh_con(1);
Mtt(3:6,tt)=Trim_var(control_idx(1:4),1);
Mtt(7:8,tt)=(Trim_var(control_idx(5:6),1));
Mtt(9,tt)=power1+power2+power3+power4+power5+power6;

Mttt=Mtt(:,1:tt)';

MMA(row1:row2, :) = A;
MMB(row1:row2, :) = B_all;

end

trim_results = struct();
trim_results.cfg = cfg;
trim_results.trim_table = Mttt;
trim_results.stability_A = MMA;
trim_results.control_B = MMB;
trim_results.final_trim_var = trim_solution_var;
trim_results.post_stability_trim_var = Trim_var;
trim_results.last_eigenvalues = eig_A;
trim_results.stability_derivatives_total = derivatives;
trim_results.stability_derivatives_by_rotor = {derivatives1, derivatives2, derivatives3, derivatives4, derivatives5, derivatives6};
trim_results.stability_derivatives_fuselage = derivatives - derivatives1 - derivatives2 - derivatives3 - derivatives4 - derivatives5 - derivatives6;
trim_results.tilt_angle_deg = tilt_angle;
trim_results.rotor_tilt_angle_deg = rotor_tilt_angles(:);
trim_results.speed_mps = cfg.trim.speed_mps(:);
trim_results.n_speed_points = numel(cfg.trim.speed_mps);
trim_results.rotor_locations_m = [
    X1_LOC(:).';
    X2_LOC(:).';
    X3_LOC(:).';
    X4_LOC(:).';
    X5_LOC(:).';
    X6_LOC(:).'];
assignin('base', 'trim_results', trim_results);

    











%hover_efficiency=forces(3)*Trim_var(2*Nb+1)/abs(forces(6)*omega);
t0 = tic;  
x0=Trim_var;

function cfg = bemtflap_default_cfg()
cfg = struct();

cfg.environment.rho_kg_m3 = 1.225;
cfg.environment.gravity_m_s2 = 9.81;

cfg.vehicle.mass_kg = 1900;
cfg.vehicle.inertia_kg_m2 = [1966.5, 5245.3, 3282.7];

cfg.rotor.radius_m = 1.3;
cfg.rotor.blade_count = 5;
cfg.rotor.omega_rad_s = 90;
cfg.rotor.flap_inertia_kg_m2 = 2.25;
cfg.rotor.flap_spring_nm_rad = 1000;
cfg.rotor.airfoil_section_edges = [0.25 0.40 0.50 0.80 0.92];
cfg.rotor.tilt_angle_deg = [];
cfg.rotor.rotational_direction = [1 -1 1 -1 1 -1];
cfg.rotor.blade_element_count = 10;
cfg.rotor.azimuth_steps = 72;

cfg.switch.geometry = "default";      % "default" or "lookup"
cfg.switch.rotor_positions = "default"; % "default" or "lookup"
cfg.switch.chord = "default";         % "default", "lookup", "txt", or "mat"
cfg.switch.pretwist = "default";      % "default", "lookup", "txt", or "mat"
cfg.switch.airfoil = "default";       % "default" or "lookup"
cfg.switch.fuselage = "default";      % "default" or "lookup"
cfg.switch.controls = "default";      % "default" or "lookup"

cfg.initial.uvw_earth_mps = [20 5 0];
cfg.initial.pqr_rad_s = [0 0 0];
cfg.initial.fixed_wing_control = [0 0 0];

cfg.trim.tilt_angle_deg = 60;
cfg.trim.speed_mps = 40;
cfg.trim.max_iterations = 10000;
cfg.stability.max_iterations = 10000;
cfg.output.verbose = false;
cfg.trim.initial.rotor_state = [zeros(2*cfg.rotor.blade_count,1); 1];
cfg.trim.initial.collective_deg = 18;
cfg.trim.initial.longitudinal_deg = -0.002;
cfg.trim.initial.lateral_deg = 0;
cfg.trim.initial.yaw_deg = 0;
cfg.trim.initial.pitch_rad = -0.01;
cfg.trim.initial.roll_rad = 0;

cfg.defaults.geometry.x_cg_mm = 3000;
cfg.defaults.geometry.y_cg_mm = 0;
cfg.defaults.geometry.z_cg_mm = -950;
cfg.defaults.geometry.first_rotor_x_mm = 430;
cfg.defaults.geometry.first_rotor_z_mm = -1900;
cfg.defaults.geometry.rotor_position_coeffs_mm = [
    3600 -700    0  6000 -1290    0 -750
    1490 -400 -140  2500 -1490  140 -450
    1490 -400 -140 -2500 -1490  140 -450
    3600 -700    0 -6000 -1290    0 -750
    5710 -400  140  2500 -1210 -140 -400
    5710 -400  140 -2500 -1210 -140 -400];
cfg.data.geometry.cg_file = 'CG_positions.txt';
cfg.data.geometry.rotor_positions_file = 'Rotor_positions.txt';

cfg.defaults.chord_m = 0.18;
cfg.defaults.pretwist_root_deg = 0;
cfg.defaults.pretwist_tip_deg = -12;
cfg.data.chord.txt_file = 'Chord.txt';
cfg.data.chord.mat_file = 'chord_interp.mat';
cfg.data.chord.mat_var = 'F';
cfg.data.pretwist.txt_file = 'Pretwist.txt';
cfg.data.pretwist.mat_file = 'pretwist_interp.mat';
cfg.data.pretwist.mat_var = 'pre_twist';
cfg.defaults.airfoil.cl_alpha_per_rad = 2*pi;
cfg.defaults.airfoil.cl_max = 1.45;
cfg.defaults.airfoil.cd0 = 0.012;
cfg.defaults.airfoil.cd_alpha2 = 0.08;

cfg.defaults.fuselage.cd0 = 0.08;
cfg.defaults.fuselage.cl_alpha_per_rad = 2.5;
cfg.defaults.fuselage.cm_alpha_per_rad = -0.15;
cfg.defaults.fuselage.cc_beta = -0.7;
cfg.defaults.fuselage.cn_beta = 0.08;
cfg.defaults.fuselage.cll_beta = -0.08;
cfg.fuselage.reference_area_m2 = 12;
cfg.fuselage.mean_aero_chord_m = 1;
cfg.fuselage.span_m = 10;

cfg.defaults.controls.elevator = [0, 0.40, -0.70];
cfg.defaults.controls.rudder = [0.25, -0.08, 0];
cfg.defaults.controls.aileron = [0, 0, 0.12];
cfg.data.fuselage.reference_point_mm = [3600, 0, 0];
end

function cfg = bemtflap_complete_cfg(cfg)
defaults = bemtflap_default_cfg();
cfg = merge_struct(defaults, cfg);
end

function out = merge_struct(base, override)
out = base;
fields = fieldnames(override);
for ii = 1:numel(fields)
    name = fields{ii};
    if isstruct(override.(name)) && isfield(out, name) && isstruct(out.(name))
        out.(name) = merge_struct(out.(name), override.(name));
    else
        out.(name) = override.(name);
    end
end
end

function tilt_angle = validate_airframe_tilt_angle(raw)
raw = raw(:).';
if ~isscalar(raw) || ~isfinite(raw)
    error('BEMTFLAP:BadTiltAngle', ...
        'cfg.trim.tilt_angle_deg must be one scalar airframe/lookup tilt angle. Use cfg.rotor.tilt_angle_deg for per-rotor tilts.');
end
tilt_angle = raw;
end

function rotor_tilt_angles = setup_rotor_tilt_angles(cfg, airframe_tilt_angle, n_rotors)
if isfield(cfg, 'rotor') && isfield(cfg.rotor, 'tilt_angle_deg') && ~isempty(cfg.rotor.tilt_angle_deg)
    rotor_tilt_angles = cfg.rotor.tilt_angle_deg(:).';
else
    rotor_tilt_angles = airframe_tilt_angle;
end
rotor_tilt_angles = expand_to_rotors(rotor_tilt_angles, n_rotors, 'cfg.rotor.tilt_angle_deg');
if any(~isfinite(rotor_tilt_angles))
    error('BEMTFLAP:BadTiltAngle', 'cfg.rotor.tilt_angle_deg contains non-finite values.');
end
end

function tf = diagnostic_trim_only(cfg)
tf = false;
if isfield(cfg, 'diagnostic') && isfield(cfg.diagnostic, 'trim_only')
    tf = logical(cfg.diagnostic.trim_only);
end
end

function ref_m = setup_fuselage_reference(cfg)
ref_mm = [3600, 0, 0];
if isfield(cfg, 'data') && isfield(cfg.data, 'fuselage') && ...
        isfield(cfg.data.fuselage, 'reference_point_mm')
    ref_mm = cfg.data.fuselage.reference_point_mm;
elseif isfield(cfg, 'data') && isfield(cfg.data, 'fuselage') && ...
        isfield(cfg.data.fuselage, 'reference_point_m')
    ref_m = cfg.data.fuselage.reference_point_m;
    ref_m = ref_m(:).';
    if numel(ref_m) ~= 3 || any(~isfinite(ref_m))
        error('BEMTFLAP:BadFuselageReference', ...
            'cfg.data.fuselage.reference_point_m must be [x y z] in meters.');
    end
    return;
end
ref_mm = ref_mm(:).';
if numel(ref_mm) ~= 3 || any(~isfinite(ref_mm))
    error('BEMTFLAP:BadFuselageReference', ...
        'cfg.data.fuselage.reference_point_mm must be [x y z] in millimeters.');
end
ref_m = ref_mm / 1000;
end

function geom = setup_fuselage_geometry(cfg)
geom.reference_area_m2 = cfg.fuselage.reference_area_m2;
geom.mean_aero_chord_m = cfg.fuselage.mean_aero_chord_m;
geom.span_m = cfg.fuselage.span_m;
if any(~isfinite([geom.reference_area_m2, geom.mean_aero_chord_m, geom.span_m])) || ...
        any([geom.reference_area_m2, geom.mean_aero_chord_m, geom.span_m] <= 0)
    error('BEMTFLAP:BadFuselageGeometry', ...
        'cfg.fuselage reference area, mean aerodynamic chord, and span must be positive finite values.');
end
end

function opts = setup_rotor_bemt_options(cfg, rho)
opts.rho_kg_m3 = rho;
opts.blade_element_count = cfg.rotor.blade_element_count;
opts.azimuth_steps = cfg.rotor.azimuth_steps;
if any(~isfinite([opts.rho_kg_m3, opts.blade_element_count, opts.azimuth_steps])) || ...
        opts.rho_kg_m3 <= 0 || opts.blade_element_count < 1 || opts.azimuth_steps < 4
    error('BEMTFLAP:BadRotorBemtOptions', ...
        'Rotor BEMT rho, blade_element_count, and azimuth_steps must be valid positive values.');
end
opts.blade_element_count = round(opts.blade_element_count);
opts.azimuth_steps = round(opts.azimuth_steps);
end

function dirs = setup_rotational_direction(cfg, n_rotors)
dirs = cfg.rotor.rotational_direction(:).';
dirs = expand_to_rotors(dirs, n_rotors, 'cfg.rotor.rotational_direction');
if any(~isfinite(dirs)) || any(abs(dirs) ~= 1)
    error('BEMTFLAP:BadRotationalDirection', ...
        'cfg.rotor.rotational_direction must contain +1 or -1 for each rotor.');
end
end

function geom = setup_default_rotor_geometry(cfg)
geom.coeffs_mm = cfg.defaults.geometry.rotor_position_coeffs_mm;
if ~isequal(size(geom.coeffs_mm), [6 7]) || any(~isfinite(geom.coeffs_mm(:)))
    error('BEMTFLAP:BadRotorPositionDefaults', ...
        'cfg.defaults.geometry.rotor_position_coeffs_mm must be a finite 6x7 matrix.');
end
end

function [x_cg, y_cg, z_cg, first_rotor_x, first_rotor_z, rotor_position_lookup] = setup_legacy_geometry(cfg, data_dir)
rotor_position_lookup = [];
geom = cfg.defaults.geometry;
x_cg = @(tilt) geom.x_cg_mm + 0.*tilt;
y_cg = @(tilt) geom.y_cg_mm + 0.*tilt;
z_cg = @(tilt) geom.z_cg_mm + 0.*tilt;
first_rotor_x = @(tilt) geom.first_rotor_x_mm + 0.*tilt;
first_rotor_z = @(tilt) geom.first_rotor_z_mm + 0.*tilt;
if is_lookup_mode(cfg.switch.geometry)
    cg_file = fullfile(data_dir, 'CG_positions.txt');
    if isfield(cfg, 'data') && isfield(cfg.data, 'geometry') && isfield(cfg.data.geometry, 'cg_file')
        cg_file = fullfile(data_dir, cfg.data.geometry.cg_file);
    end
    if exist(cg_file, 'file') == 2
        cg_lookup = build_cg_lookup_from_txt(cg_file, 'linear');
        x_cg = cg_lookup.x;
        y_cg = cg_lookup.y;
        z_cg = cg_lookup.z;
    else
        x_cg = load_mat_object(data_dir, 'x_cg.mat', 'x_cg');
        z_cg = load_mat_object(data_dir, 'z_cg.mat', 'z_cg');
    end
    if ~is_lookup_mode(cfg.switch.rotor_positions)
        first_rotor_x_file = fullfile(data_dir, 'first_rotor_x.mat');
        first_rotor_z_file = fullfile(data_dir, 'first_rotor_z.mat');
        if exist(first_rotor_x_file, 'file') == 2
            first_rotor_x = load_mat_object(data_dir, 'first_rotor_x.mat', 'first_rotor_x');
        end
        if exist(first_rotor_z_file, 'file') == 2
            first_rotor_z = load_mat_object(data_dir, 'first_rotor_z.mat', 'first_rotor_z');
        end
    end
end

if is_lookup_mode(cfg.switch.rotor_positions)
    rotor_positions_file = fullfile(data_dir, 'Rotor_positions.txt');
    if isfield(cfg, 'data') && isfield(cfg.data, 'geometry') && isfield(cfg.data.geometry, 'rotor_positions_file')
        rotor_positions_file = fullfile(data_dir, cfg.data.geometry.rotor_positions_file);
    end
    if exist(rotor_positions_file, 'file') ~= 2
        error('BEMTFLAP:MissingData', 'Selected rotor position lookup, but file not found: %s', rotor_positions_file);
    end
    rotor_position_lookup = build_rotor_position_lookup_from_txt(rotor_positions_file, 6, 'linear');
end
end

function [F, pre_twist, cd1, cd2, cd3, cd4, cd5, cd6, cl1, cl2, cl3, cl4, cl5, cl6] = setup_rotor_model(cfg, data_dir)
R = cfg.rotor.radius_m;
F = setup_chord_lookup(cfg, data_dir, R);
pre_twist = setup_pretwist_lookup(cfg, data_dir);

if is_lookup_mode(cfg.switch.airfoil)
    cd1 = CS1_cd_lookup([], [], fullfile(data_dir, 'CS1_cd.txt')).F;
    cd2 = CS1_cd_lookup([], [], fullfile(data_dir, 'CS2_cd.txt')).F;
    cd3 = CS1_cd_lookup([], [], fullfile(data_dir, 'CS3_cd.txt')).F;
    cd4 = CS1_cd_lookup([], [], fullfile(data_dir, 'CS4_cd.txt')).F;
    cd5 = CS1_cd_lookup([], [], fullfile(data_dir, 'CS5_cd.txt')).F;
    cd6 = CS1_cd_lookup([], [], fullfile(data_dir, 'CS6_cd.txt')).F;
    cl1 = CS1_cd_lookup([], [], fullfile(data_dir, 'CS1_cl.txt')).F;
    cl2 = CS1_cd_lookup([], [], fullfile(data_dir, 'CS2_cl.txt')).F;
    cl3 = CS1_cd_lookup([], [], fullfile(data_dir, 'CS3_cl.txt')).F;
    cl4 = CS1_cd_lookup([], [], fullfile(data_dir, 'CS4_cl.txt')).F;
    cl5 = CS1_cd_lookup([], [], fullfile(data_dir, 'CS5_cl.txt')).F;
    cl6 = CS1_cd_lookup([], [], fullfile(data_dir, 'CS6_cl.txt')).F;
else
    cl_fun = @(alpha_deg, mach) default_rotor_cl(alpha_deg, mach, cfg.defaults.airfoil);
    cd_fun = @(alpha_deg, mach) default_rotor_cd(alpha_deg, mach, cfg.defaults.airfoil);
    cd1 = cd_fun; cd2 = cd_fun; cd3 = cd_fun; cd4 = cd_fun; cd5 = cd_fun; cd6 = cd_fun;
    cl1 = cl_fun; cl2 = cl_fun; cl3 = cl_fun; cl4 = cl_fun; cl5 = cl_fun; cl6 = cl_fun;
end
end

function F = setup_chord_lookup(cfg, data_dir, R)
mode = lower(string(cfg.switch.chord));
txt_file = fullfile(data_dir, optional_data_filename(cfg, 'chord', 'txt_file', 'Chord.txt'));
mat_file = optional_data_filename(cfg, 'chord', 'mat_file', 'chord_interp.mat');
mat_var = optional_data_filename(cfg, 'chord', 'mat_var', 'F');

if is_default_mode(mode)
    chord_m = cfg.defaults.chord_m;
    F = @(x) chord_m + 0.*x;
elseif is_text_lookup_mode(mode)
    lookup = build_1d_lookup_from_txt(txt_file, 'chord_m', 'linear');
    F = lookup.eval;
elseif is_mat_lookup_mode(mode)
    raw = load_mat_object(data_dir, mat_file, mat_var);
    raw = set_extrapolation_if_supported(raw);
    F = @(x) R .* raw(x);
elseif is_lookup_mode(mode)
    if exist(txt_file, 'file') == 2
        lookup = build_1d_lookup_from_txt(txt_file, 'chord_m', 'linear');
        F = lookup.eval;
    else
        raw = load_mat_object(data_dir, mat_file, mat_var);
        raw = set_extrapolation_if_supported(raw);
        F = @(x) R .* raw(x);
    end
else
    error('BEMTFLAP:BadSwitch', 'cfg.switch.chord must be "default", "lookup", "txt", or "mat".');
end
end

function pre_twist = setup_pretwist_lookup(cfg, data_dir)
mode = lower(string(cfg.switch.pretwist));
txt_file = fullfile(data_dir, optional_data_filename(cfg, 'pretwist', 'txt_file', 'Pretwist.txt'));
mat_file = optional_data_filename(cfg, 'pretwist', 'mat_file', 'pretwist_interp.mat');
mat_var = optional_data_filename(cfg, 'pretwist', 'mat_var', 'pre_twist');

if is_default_mode(mode)
    root_twist = cfg.defaults.pretwist_root_deg;
    tip_twist = cfg.defaults.pretwist_tip_deg;
    pre_twist = @(x) root_twist + (tip_twist - root_twist).*x;
elseif is_text_lookup_mode(mode)
    lookup = build_1d_lookup_from_txt(txt_file, 'pretwist_deg', 'linear');
    pre_twist = lookup.eval;
elseif is_mat_lookup_mode(mode)
    pre_twist = load_mat_object(data_dir, mat_file, mat_var);
    pre_twist = set_extrapolation_if_supported(pre_twist);
elseif is_lookup_mode(mode)
    if exist(txt_file, 'file') == 2
        lookup = build_1d_lookup_from_txt(txt_file, 'pretwist_deg', 'linear');
        pre_twist = lookup.eval;
    else
        pre_twist = load_mat_object(data_dir, mat_file, mat_var);
        pre_twist = set_extrapolation_if_supported(pre_twist);
    end
else
    error('BEMTFLAP:BadSwitch', 'cfg.switch.pretwist must be "default", "lookup", "txt", or "mat".');
end
end

function [cd, cl, cm, cc, cn, cll, elev, rudd, airp] = setup_fuselage_model(cfg, data_dir)
if is_lookup_mode(cfg.switch.fuselage)
    cd = build_C_lookup_from_txt(fullfile(data_dir, 'Fuselage_cd.txt'), 'linear', false);
    cl = build_C_lookup_from_txt(fullfile(data_dir, 'Fuselage_cl.txt'), 'linear', false);
    cm = build_C_lookup_from_txt(fullfile(data_dir, 'Fuselage_cm.txt'), 'linear', false);
    cc = build_C_lookup_from_txt(fullfile(data_dir, 'Fuselage_cc.txt'), 'linear', false);
    cn = build_C_lookup_from_txt(fullfile(data_dir, 'Fuselage_cn.txt'), 'linear', false);
    cll = build_C_lookup_from_txt(fullfile(data_dir, 'Fuselage_cll.txt'), 'linear', false);
else
    fuse = cfg.defaults.fuselage;
    cd.CD = @(tilt, alpha_deg) fuse.cd0 + 0.*tilt + 0.*alpha_deg;
    cl.CD = @(tilt, alpha_deg) fuse.cl_alpha_per_rad.*deg2rad(alpha_deg) + 0.*tilt;
    cm.CD = @(tilt, alpha_deg) fuse.cm_alpha_per_rad.*deg2rad(alpha_deg) + 0.*tilt;
    cc.CD = @(tilt, alpha_deg) fuse.cc_beta + 0.*tilt + 0.*alpha_deg;
    cn.CD = @(tilt, alpha_deg) fuse.cn_beta + 0.*tilt + 0.*alpha_deg;
    cll.CD = @(tilt, alpha_deg) fuse.cll_beta + 0.*tilt + 0.*alpha_deg;
end

if is_lookup_mode(cfg.switch.controls)
    elev = build_fuselage_elevator_lookup(fullfile(data_dir, 'Fuselage_elevator.txt'), 'linear', false);
    rudd = build_fuselage_elevator_lookup(fullfile(data_dir, 'Fuselage_rudder.txt'), 'linear', false);
    airp = build_fuselage_elevator_lookup(fullfile(data_dir, 'Fuselage_roll.txt'), 'linear', false);
else
    ctrl = cfg.defaults.controls;
    elev.eval = @(deflection, alpha_deg) default_control_delta(deflection, alpha_deg, ctrl.elevator);
    rudd.eval = @(deflection, alpha_deg) default_control_delta(deflection, alpha_deg, ctrl.rudder);
    airp.eval = @(deflection, alpha_deg) default_control_delta(deflection, alpha_deg, ctrl.aileron);
end
end

function tf = is_default_mode(mode)
mode = lower(string(mode));
tf = any(mode == ["default", "builtin", "analytic"]);
end

function tf = is_lookup_mode(mode)
mode = lower(string(mode));
tf = any(mode == ["lookup", "legacy_lookup", "table", "tables"]);
end

function tf = is_text_lookup_mode(mode)
mode = lower(string(mode));
tf = any(mode == ["txt", "text", "text_lookup"]);
end

function tf = is_mat_lookup_mode(mode)
mode = lower(string(mode));
tf = any(mode == ["mat", "mat_lookup", "legacy_lookup"]);
end

function values = expand_to_rotors(values, n_rotors, label)
values = values(:).';
if isscalar(values)
    values = repmat(values, 1, n_rotors);
elseif numel(values) ~= n_rotors
    error('BEMTFLAP:BadRotorVector', '%s must be scalar or contain %d values.', label, n_rotors);
end
end

function filename = optional_data_filename(cfg, group, field, default_filename)
filename = default_filename;
if isfield(cfg, 'data') && isfield(cfg.data, group) && isfield(cfg.data.(group), field)
    filename = cfg.data.(group).(field);
end
end

function obj = load_mat_object(data_dir, filename, preferred_name)
path = fullfile(data_dir, filename);
if exist(path, 'file') ~= 2
    error('BEMTFLAP:MissingData', 'Required lookup file not found: %s', path);
end
loaded = load(path);
if isfield(loaded, preferred_name)
    obj = loaded.(preferred_name);
    return;
end
names = fieldnames(loaded);
if isempty(names)
    error('BEMTFLAP:MissingData', 'No variables were found in %s.', path);
end
obj = loaded.(names{1});
end

function obj = set_extrapolation_if_supported(obj)
try
    obj.ExtrapolationMethod = 'linear';
catch
end
end

function edges = validate_airfoil_section_edges(edges)
edges = edges(:).';
if numel(edges) ~= 5
    error('BEMTFLAP:BadAirfoilSections', ...
        'cfg.rotor.airfoil_section_edges must contain five nondimensional cutoff locations for six airfoil sections.');
end
if any(~isfinite(edges)) || any(edges <= 0) || any(edges >= 1) || any(diff(edges) <= 0)
    error('BEMTFLAP:BadAirfoilSections', ...
        'cfg.rotor.airfoil_section_edges must be finite, strictly increasing, and between 0 and 1.');
end
end

function cl = default_rotor_cl(alpha_deg, mach, airfoil)
cl = airfoil.cl_alpha_per_rad.*deg2rad(alpha_deg);
cl = max(min(cl, airfoil.cl_max), -airfoil.cl_max);
cl = cl + 0.*mach;
end

function cd = default_rotor_cd(alpha_deg, mach, airfoil)
alpha_rad = deg2rad(alpha_deg);
cd = airfoil.cd0 + airfoil.cd_alpha2.*alpha_rad.^2 + 0.*mach;
end

function vals = default_control_delta(deflection, alpha_deg, coeff_per_rad)
vals = deg2rad(deflection).*coeff_per_rad + 0.*alpha_deg;
end

function indices = get_stability_derivative_indices(cfg)
indices = 1:10;
if isfield(cfg, 'stability') && isfield(cfg.stability, 'derivative_indices')
    indices = cfg.stability.derivative_indices;
end
indices = validate_derivative_indices(indices, 1, 10, 'cfg.stability.derivative_indices');
end

function indices = get_control_derivative_indices(cfg)
indices = 11:13;
if isfield(cfg, 'stability') && isfield(cfg.stability, 'control_derivative_indices')
    indices = cfg.stability.control_derivative_indices;
end
indices = validate_derivative_indices(indices, 11, 13, 'cfg.stability.control_derivative_indices');
end

function indices = validate_derivative_indices(indices, lower_bound, upper_bound, label)
indices = indices(:).';
if isempty(indices)
    return;
end
if any(~isfinite(indices)) || any(indices ~= round(indices)) || any(indices < lower_bound) || any(indices > upper_bound)
    error('BEMTFLAP:BadDerivativeIndices', ...
        '%s must contain integer values from %d to %d.', label, lower_bound, upper_bound);
end
end

function tf = use_previous_trim_solution(cfg)
tf = true;
if isfield(cfg, 'trim') && isfield(cfg.trim, 'use_previous_solution')
    tf = logical(cfg.trim.use_previous_solution);
end
end

function Trim_var = apply_initial_trim(Trim_var, cfg)
init = cfg.trim.initial;
if isfield(init, 'vector') && numel(init.vector) == numel(Trim_var)
    Trim_var = init.vector(:);
    return;
end
if isfield(init, 'vector_72') && numel(init.vector_72) == numel(Trim_var)
    Trim_var = init.vector_72(:);
    return;
end

rotor_state_size = (numel(Trim_var)-6)/6;
rotor_state = [];
if isfield(init, 'rotor_state') && numel(init.rotor_state) == rotor_state_size
    rotor_state = init.rotor_state(:);
elseif isfield(init, 'rotor_state_11') && numel(init.rotor_state_11) == rotor_state_size
    rotor_state = init.rotor_state_11(:);
end

if ~isempty(rotor_state)
    for rotor_number = 1:6
        idx = (rotor_number-1)*rotor_state_size + (1:rotor_state_size);
        Trim_var(idx, 1) = rotor_state;
    end
end

control_idx = 6*rotor_state_size + (1:6);
if isfield(init, 'collective_deg'), Trim_var(control_idx(1)) = init.collective_deg; end
if isfield(init, 'longitudinal_deg'), Trim_var(control_idx(2)) = init.longitudinal_deg; end
if isfield(init, 'lateral_deg'), Trim_var(control_idx(3)) = init.lateral_deg; end
if isfield(init, 'yaw_deg'), Trim_var(control_idx(4)) = init.yaw_deg; end
if isfield(init, 'pitch_rad'), Trim_var(control_idx(5)) = init.pitch_rad; end
if isfield(init, 'roll_rad'), Trim_var(control_idx(6)) = init.roll_rad; end
end


function [errors,forces, beta_vals,dbeta_vals,power,ATT]=bemt_flapp(Trim_var, R,Nb, omega,I_beta, velo, k_beta,F,pre_twist,cd1,cd2,cd3,cd4,cd5,cd6,cl1,cl2,cl3,cl4,cl5,cl6,airfoil_section_edges,theta00,tilt_angle,angular_speed,rotational_direction,acceleration,angular_acc,rotor_bemt_options,Hub_loc)
% Parameters

rho = rotor_bemt_options.rho_kg_m3;
N_BE = rotor_bemt_options.blade_element_count;
Steps = rotor_bemt_options.azimuth_steps;
ATT=zeros(Steps,N_BE);
%velo(2)=velo(2)*rotational_direction;
%angular_speed(1)=angular_speed(1)*rotational_direction;
%angular_speed(3)=angular_speed(3)*rotational_direction;
veloo=velo+cross(angular_speed,Hub_loc');         
% if (angular_speed(2)>0.02)
%     velo
%     angular_speed(2)
%     veloo
% end
tilt_conversion=[sin(deg2rad(tilt_angle)) 0 cos(deg2rad(tilt_angle));0 1 0;-cos(deg2rad(tilt_angle)) 0 sin(deg2rad(tilt_angle))];
inv_tiltconversion=inv(tilt_conversion);
v12=tilt_conversion*veloo;
w_disc=tilt_conversion*angular_speed;
a_disc=tilt_conversion*acceleration;
ang_ace=tilt_conversion*angular_acc;
%%v12(2)=v12(2)*rotational_direction;
%%w_disc(1)=w_disc(1)*rotational_direction;
%%w_disc(3)=w_disc(3)*rotational_direction;
omega=omega*rotational_direction;
ppp=w_disc(1);
qqq=w_disc(2);
rrr=w_disc(3);

uuu=v12(1);
vvv=v12(2);
www=v12(3);

aau=a_disc(1);
aav=a_disc(2);
aaw=a_disc(3);

aap=ang_ace(1);
aaq=ang_ace(2);
aar=ang_ace(3);

theta0=deg2rad(theta00);
                                   
dr = (R-0.15*R) / N_BE;                              
vi=Trim_var(end);

%k_beta = 0;  % Assuming zero stiffness

%beta0 = 0.15;
%beta0_dot = 0.08;
%beta = beta0;
%beta_dot = beta0_dot;
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
%dbeta_vals(:)=beta0_dot;

Tb_average=0;
Hb_average=0;
Sb_average=0;
Torque_average=0;
M_average=0;
L_average=0;
MMde=zeros(Steps,N_BE);
MMli=zeros(Steps,N_BE);
for k=1:Nb
    beta_dot=dbeta_vals(1,k);
    for Az1=1:Steps
	    Tb_new = 0;
    	M_A=0;
        Hb_new =0;
        Sb_new=0;
        M_torque=0;
	    Az=rotational_direction*((Az1-1)*dpsi+(k-1)*2*pi/Nb);
        azimuth_new=[-cos(Az) -sin(Az) 0; sin(Az) -cos(Az) 0;0 0 1];

        azimuth_new2=inv(azimuth_new);
        blade_m2=[cos(beta_vals(Az1,k)) 0 -sin(beta_vals(Az1,k)); 0 1 0;sin(beta_vals(Az1,k)) 0 cos(beta_vals(Az1,k))];%%%???
        %blade_m2=[1 0 0; 0 cos(beta_vals(Az1,k)) -sin(beta_vals(Az1,k)); 0 sin(beta_vals(Az1,k)) cos(beta_vals(Az1,k))];
        %blade_m2=[1 0 0 ; 0 1 0; 0 0 1];
        blade_m=inv(blade_m2);
        velocity2=azimuth_new*velocity1;
        %w_shaft=azimuth_new*w_disc+1*[0;0;omega];
        w_shaft=azimuth_new*w_disc+1*[0;0;omega];
        w_blade=blade_m2*w_shaft+[0;dbeta_vals(Az1,k);0];
        velocity=blade_m2*velocity2;
        for z = 1:N_BE
            x = 0.15+ (0.85 / N_BE /2) + (z - 1) * (0.85 / N_BE); 
            r = x * R;
            v_blade_elem=velocity+cross(w_blade,[r;0;0]);

            theta = theta0+deg2rad(pre_twist(x));
            V_n=v_blade_elem(3)-vi*cos(beta_vals(Az1,k));   
            vt=rotational_direction*v_blade_elem(2);
            phi_new = atan2(V_n , vt);
            alpha_new = theta + phi_new; 
            if (k==1)
                ATT(Az1,z)=alpha_new;
            end

            Mach=sqrt(V_n^2+vt^2)/340;
            if (x<airfoil_section_edges(1))
                C_Lnew = cl1(rad2deg(alpha_new),Mach);
                C_Dnew = cd1(rad2deg(alpha_new),Mach);
            elseif (x<airfoil_section_edges(2))
                C_Lnew = cl2(rad2deg(alpha_new),Mach);
                C_Dnew = cd2(rad2deg(alpha_new),Mach); 
            elseif (x<airfoil_section_edges(3))
                C_Lnew = cl3(rad2deg(alpha_new),Mach);
                C_Dnew = cd3(rad2deg(alpha_new),Mach);
            elseif (x<airfoil_section_edges(4))
                C_Lnew = cl4(rad2deg(alpha_new),Mach);
                C_Dnew = cd4(rad2deg(alpha_new),Mach);   
            elseif (x<airfoil_section_edges(5))
                C_Lnew = cl5(rad2deg(alpha_new),Mach);
                C_Dnew = cd5(rad2deg(alpha_new),Mach);  
            else
                C_Lnew = cl6(rad2deg(alpha_new),Mach);
                C_Dnew = cd6(rad2deg(alpha_new),Mach);
            end 
            %C_Lnew=6.0*alpha_new;
            %C_Dnew=0.002+rad2deg(alpha_new)^2*0.00004;
               
            dL_new = 0.5 * rho * (vt^2+(vi-V_n)^2) * F(x) * dr * C_Lnew;
            dD_new = 0.5 * rho * (vt^2+(vi-V_n)^2) * F(x) * dr * C_Dnew;
            if k==1
                MMde(Az1,z)=dD_new;
                MMli(Az1,z)=dL_new;
            end
            
            T_be=-(dL_new * cos(phi_new) + dD_new * sin(phi_new));
            D_be=-rotational_direction*dD_new * cos(phi_new) + rotational_direction*dL_new * sin(phi_new);
            M_torque=M_torque+D_be*r;
            blade_force=[0; D_be; T_be];
            blade_force=blade_m*blade_force;
            blade_force=azimuth_new2*blade_force;


            Tb_new = Tb_new + blade_force(3);
            Hb_new = Hb_new + blade_force(1);
            Sb_new = Sb_new + blade_force(2);
            
            M_A= blade_force(3)*r+M_A;
        end
        Tb_average=Tb_average+Tb_new/Steps;
        Hb_average=Hb_average+Hb_new/Steps;
        Sb_average=Sb_average+Sb_new/Steps;
        Torque_average=Torque_average+M_torque/Steps;
        M_CF=-omega^2*(I_beta)*(beta_vals(Az1,k));
        M_R=-k_beta*beta_vals(Az1,k);
        M_cor=-2*I_beta*(ppp*omega*cos(Az)-qqq*omega*sin(Az));
        M_ba=I_beta*(aap*sin(Az)+aaq*cos(Az));
        M_bl=1.5*(aaw-uuu*qqq+ppp*vvv);
        M_average=M_average+M_R*cos(Az)/Steps;
        L_average=L_average+M_R*sin(Az)/Steps;

        beta_2dot=(M_A+M_CF+M_R+M_cor+M_ba+M_bl)/I_beta;
        beta_dot = beta_dot + beta_2dot * dt;
        dbeta_vals(Az1+1,k)= beta_dot;
        beta_vals(Az1+1,k) = beta_vals(Az1,k) + beta_dot * dt;
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

function forces=fuselage_aerodynamics(X_FUSELAGE_REF,vehicle_control,roe,uvw,pqr,tilt_angle, cd,cl, cm,cc,cn,cll,elev,rudd, airp,fuselage_geometry)

S = fuselage_geometry.reference_area_m2;
C_a = fuselage_geometry.mean_aero_chord_m;
b = fuselage_geometry.span_m;

U=uvw+cross(pqr',X_FUSELAGE_REF');

velocity=sqrt(U(1)^2+U(2)^2+U(3)^2);

alpha1=atan2(U(3),U(1));

beta1=atan2(-U(2),U(1));

alpha=rad2deg(atan2(U(3),U(1)));

beta=rad2deg(atan2(-U(2),U(1)));




cd_cof=cd.CD(tilt_angle,alpha);


cl_cof=cl.CD(tilt_angle,alpha);


cm_cof=cm.CD(tilt_angle,alpha);


cc_cof=cc.CD(tilt_angle,alpha);


cn_cof=cn.CD(tilt_angle,alpha);


cll_cof=cll.CD(tilt_angle,alpha);


vals = elev.eval(vehicle_control(1), alpha); 

dcd= vals(1);

dcl= vals(2);

dcm= vals(3);



vals = rudd.eval(vehicle_control(2), alpha); 

dcc_ru= vals(1);

dcn_ru= vals(2);

dcll_ru= vals(3);



vals = airp.eval(vehicle_control(3), alpha); 

dcc_ap= vals(1);

dcn_ap= vals(2);

dcll_ap= vals(3);





l=0.5*roe*velocity^2*S*(cl_cof+dcl);

d=0.5*roe*velocity^2*S*(cd_cof+dcd);

m=0.5*roe*velocity^2*S*C_a*(cm_cof+dcm);

c=0.5*roe*velocity^2*S*(cc_cof/5.0*beta+dcc_ru+dcc_ap);

n=0.5*roe*velocity^2*S*b*(cn_cof/5.0*beta+dcn_ru+dcn_ap);

ll=0.5*roe*velocity^2*S*b*(cll_cof/5.0*beta+dcll_ru+dcll_ap);

Fx_wind=[-d;-c;-l];

wind2body=[cos(alpha1)*cos(beta1) -cos(alpha1)*sin(beta1) -sin(alpha1); sin(beta1) cos(beta1) 0; sin(alpha1)*cos(beta1) -sin(alpha1)*sin(beta1) cos(alpha1)];

Fx_body=wind2body*Fx_wind;

Moment=cross(X_FUSELAGE_REF,Fx_body');

forces=[Fx_body(1); Fx_body(2); Fx_body(3); ll+Moment(1); m+Moment(2); n+Moment(3)];




end

function uvw_body=earth2body(uvw,Trim_var,set_pitch)
theta=Trim_var(end-1);
phi=Trim_var(end);
psi=0;
cphi = cos(phi);   sphi = sin(phi);
cth  = cos(theta); sth  = sin(theta);
cpsi = cos(psi);   spsi = sin(psi);

C_e2b = [ cth*cpsi,                    cth*spsi,                   -sth;
          sphi*sth*cpsi - cphi*spsi,   sphi*sth*spsi + cphi*cpsi,  sphi*cth;
          cphi*sth*cpsi + sphi*spsi,   cphi*sth*spsi - sphi*cpsi,  cphi*cth ];
uvw_body=C_e2b*uvw;
end

function [theta0_1,theta0_2,theta0_3,theta0_4,theta0_5,theta0_6]=control_allocation(Trim_var)

control_start = numel(Trim_var) - 5;
collective = Trim_var(control_start);
longitudinal = Trim_var(control_start+1);
lateral = Trim_var(control_start+2);
yaw = Trim_var(control_start+3);

theta0_1=collective+lateral;

theta0_2=collective+yaw+longitudinal;

theta0_3=collective-yaw+longitudinal;

theta0_4=collective-lateral;

theta0_5=collective-yaw-longitudinal;

theta0_6=collective+yaw-longitudinal;

end


function dx = newton_step_schur(J, r)


    nblk = 6;   ny = 6; nx = size(J,1) - ny; bsz = nx / nblk;
    if abs(bsz - round(bsz)) > eps
        error('BEMTFLAP:BadStateSize', 'Rotor state block size must be an integer.');
    end
    bsz = round(bsz);

    % Partition
    A = J(1:nx, 1:nx);
    B = J(1:nx, nx+1:nx+ny);
    C = J(nx+1:nx+ny, 1:nx);
    D = J(nx+1:nx+ny, nx+1:nx+ny);

    rx = r(1:nx);
    ry = r(nx+1:nx+ny);

    % Preallocate
    u  = zeros(nx,1);        % A^{-1}*rx
    V  = zeros(nx,ny);       % A^{-1}*B

    % Solve block-by-block using LU with pivoting
    for k = 1:nblk
        idx = (k-1)*bsz + (1:bsz);
        Ak  = A(idx, idx);
        [L,U,P] = lu(Ak);   % P*Ak = L*U

        % u_k = Ak \ rx_k
        bk = rx(idx);
        y  = L \ (P*bk);
        u(idx) = U \ y;

        % V_k = Ak \ B_k  (multiple RHS)
        Bk = B(idx, :);
        Y  = L \ (P*Bk);
        V(idx,:) = U \ Y;
    end

    % Schur complement (6x6)
    S = D - C*V;
    rhs_y = -ry + C*u;

    % Solve for dy
    dy = S \ rhs_y;

    % Back-substitute for dx
    dx_x = -(u + V*dy);

    dx = [dx_x; dy];
end

function dx = solve_bordered_jacobian(J, neg_F)
 

    nblk = 6; ny = 6; nx = size(J,1) - ny; bsz = nx / nblk;
    if abs(bsz - round(bsz)) > eps
        error('BEMTFLAP:BadStateSize', 'Rotor state block size must be an integer.');
    end
    bsz = round(bsz);

    B  = J(1:nx, nx+1:nx+ny);
    C  = J(nx+1:nx+ny, 1:nx);
    D  = J(nx+1:nx+ny, nx+1:nx+ny);
    
    F1 = neg_F(1:nx);
    F2 = neg_F(nx+1:nx+ny);

    A_inv_B  = zeros(nx, ny);
    A_inv_F1 = zeros(nx, 1);

    for i = 1:nblk
        idx = (i-1)*bsz + 1 : i*bsz; 
        
        A_i = J(idx, idx);
        
        A_inv_B(idx, :) = A_i \ B(idx, :);
        A_inv_F1(idx)   = A_i \ F1(idx);
    end

    S = D - C * A_inv_B;
    
    rhs2 = F2 - C * A_inv_F1;
    
    dx2 = S \ rhs2;

    % dx1 = A^{-1} * F1 - (A^{-1} * B) * dx2
    dx1 = A_inv_F1 - A_inv_B * dx2;

    dx = [dx1; dx2];
end
function [X1_LOC, X2_LOC, X3_LOC, X4_LOC, X5_LOC, X6_LOC, X_CCG] = rotor_locations(rotor_tilt_angles, XCG, ~, ~, rotor_position_lookup, fuselage_reference_m, default_rotor_geometry)

    XCG = XCG(:).';
    if nargin < 6 || isempty(fuselage_reference_m)
        fuselage_reference_m = [2.98694, 0, -0.78715];
    end
    fuselage_reference_m = fuselage_reference_m(:).';

    rotor_tilt_angles = expand_to_rotors(rotor_tilt_angles, 6, 'rotor_tilt_angles');

    if nargin >= 5 && ~isempty(rotor_position_lookup)
        X = rotor_position_lookup.eval(rotor_tilt_angles);
    else
        X = default_rotor_positions_mm(rotor_tilt_angles, default_rotor_geometry);
    end
    X1 = X(1, :);
    X2 = X(2, :);
    X3 = X(3, :);
    X4 = X(4, :);
    X5 = X(5, :);
    X6 = X(6, :);

    % mm -> m
    X1 = X1 / 1000;
    X2 = X2 / 1000;
    X3 = X3 / 1000;
    X4 = X4 / 1000;
    X5 = X5 / 1000;
    X6 = X6 / 1000;
    X_CCG1 = fuselage_reference_m;

    % Relative to CG
    X1_LOC = local_transform(X1, XCG);
    X2_LOC = local_transform(X2, XCG);
    X3_LOC = local_transform(X3, XCG);
    X4_LOC = local_transform(X4, XCG);
    X5_LOC = local_transform(X5, XCG);
    X6_LOC = local_transform(X6, XCG);
    X_CCG=local_transform(X_CCG1,XCG);



end


function X_LOC = local_transform(X, XCG)
    X_LOC = X-XCG;
    X_LOC(1) = -X_LOC(1);
    X_LOC(2) = -X_LOC(2);
end

function [trim_var,body_angular,velo1,Veh_con] = stability_check_up(N,trim_var1,body_angular1,velo11,Veh_con11)
    trim_var=trim_var1;
    body_angular=body_angular1;
    velo1=velo11;
    Veh_con=Veh_con11;
    if N<4
        velo1(N)=velo1(N)+0.01;
    elseif N<7
        body_angular(N-3)=body_angular(N-3)+0.01;
    elseif N<11
        control_start = numel(trim_var) - 5;
        trim_var(control_start+N-7)=trim_var(control_start+N-7)+0.01;
    else
        Veh_con(N-10)=Veh_con11(N-10)+0.01;
    end
end

function [trim_var,body_angular,velo1,Veh_con] = stability_check_down(N,trim_var1,body_angular1,velo11,Veh_con11)
    trim_var=trim_var1;
    body_angular=body_angular1;
    velo1=velo11;
    Veh_con=Veh_con11;
    if N<4
        velo1(N)=velo1(N)-0.01;
    elseif N<7
        body_angular(N-3)=body_angular(N-3)-0.01;
    elseif N<11
        control_start = numel(trim_var) - 5;
        trim_var(control_start+N-7)=trim_var(control_start+N-7)-0.01;
    else
        Veh_con(N-10)=Veh_con11(N-10)-0.01;
    end



end

function X = default_rotor_positions_mm(rotor_tilt_angles, default_rotor_geometry)
rotor_tilt_angles = expand_to_rotors(rotor_tilt_angles, 6, 'rotor_tilt_angles');
t = deg2rad(rotor_tilt_angles);
coeffs = default_rotor_geometry.coeffs_mm;

X = zeros(6, 3);
for ii = 1:6
    X(ii, 1) = coeffs(ii, 1) + coeffs(ii, 2)*cos(t(ii)) + coeffs(ii, 3)*sin(t(ii));
    X(ii, 2) = coeffs(ii, 4);
    X(ii, 3) = coeffs(ii, 5) + coeffs(ii, 6)*cos(t(ii)) + coeffs(ii, 7)*sin(t(ii));
end
end
