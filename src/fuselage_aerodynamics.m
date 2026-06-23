function forces=fuselage_aerodynamics(X_FUSELAGE_REF,vehicle_control,roe,uvw,pqr,tilt_angle, cd,cl, cm,cc,cn,cll,elev,rudd, airp,fuselage_geometry)
S = fuselage_geometry.reference_area_m2;
C_a = fuselage_geometry.mean_aero_chord_m;
b = fuselage_geometry.span_m;

U=uvw(:)+cross(pqr(:),X_FUSELAGE_REF(:));
velocity=sqrt(U(1)^2+U(2)^2+U(3)^2);
alpha1=atan2(U(3),U(1));
alpha=rad2deg(atan2(U(3),U(1)));
beta1=atan2(U(2),U(1));
beta=rad2deg(beta1);
mach = velocity / 340;

cd_cof=eval_fuselage_coeff(cd,tilt_angle,alpha,beta,mach);
cl_cof=eval_fuselage_coeff(cl,tilt_angle,alpha,beta,mach);
cm_cof=eval_fuselage_coeff(cm,tilt_angle,alpha,beta,mach);
cc_cof=eval_fuselage_coeff(cc,tilt_angle,alpha,beta,mach);
cn_cof=eval_fuselage_coeff(cn,tilt_angle,alpha,beta,mach);
cll_cof=eval_fuselage_coeff(cll,tilt_angle,alpha,beta,mach);

if isfield(elev, 'physical_surfaces')
    gain = 0.5;
    if isfield(elev, 'surface_mapping')
        gain = elev.surface_mapping;
    elseif isfield(elev, 'channel_to_physical_gain')
        gain = elev.channel_to_physical_gain;
    end
    vals = eval_physical_control_surface_deltas(vehicle_control, alpha, mach, elev.physical_surfaces, gain);
    dcd = vals(1);
    dcl = vals(2);
    dcm = vals(3);
    dcc_ru = vals(4);
    dcn_ru = vals(5);
    dcll_ru = vals(6);
    dcc_ap = 0;
    dcn_ap = 0;
    dcll_ap = 0;
else
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
end

dyn_delta = zeros(1,6);
if isfield(fuselage_geometry, 'dynamic_derivatives')
    dyn_delta = fuselage_dynamic_derivative_deltas(U, pqr, velocity, C_a, b, fuselage_geometry.dynamic_derivatives);
end

l=0.5*roe*velocity^2*S*(cl_cof+dcl+dyn_delta(2));
d=0.5*roe*velocity^2*S*(cd_cof+dcd+dyn_delta(1));
m=0.5*roe*velocity^2*S*C_a*(cm_cof+dcm+dyn_delta(3));
if is_absolute_beta_coeff(cc, cn, cll)
    cc_base = cc_cof;
    cn_base = cn_cof;
    cll_base = cll_cof;
else
    cc_base = cc_cof/5.0*beta;
    cn_base = cn_cof/5.0*beta;
    cll_base = cll_cof/5.0*beta;
end

c=0.5*roe*velocity^2*S*(cc_base+dcc_ru+dcc_ap+dyn_delta(4));
n=0.5*roe*velocity^2*S*b*(cn_base+dcn_ru+dcn_ap+dyn_delta(5));
ll=0.5*roe*velocity^2*S*b*(cll_base+dcll_ru+dcll_ap+dyn_delta(6));

Fx_wind=[-d;c;-l];
wind2body=[cos(alpha1)*cos(beta1) -cos(alpha1)*sin(beta1) -sin(alpha1); sin(beta1) cos(beta1) 0; sin(alpha1)*cos(beta1) -sin(alpha1)*sin(beta1) cos(alpha1)];
Fx_body=wind2body*Fx_wind;
Moment=cross(X_FUSELAGE_REF,Fx_body');

forces=[Fx_body(1); Fx_body(2); Fx_body(3); ll+Moment(1); m+Moment(2); n+Moment(3)];
end

function val = eval_fuselage_coeff(obj, tilt_angle, alpha_deg, beta_deg, mach)
if isfield(obj, 'eval')
    val = obj.eval(tilt_angle, alpha_deg, beta_deg, mach);
else
    val = obj.CD(tilt_angle, alpha_deg);
end
end

function tf = is_absolute_beta_coeff(cc, cn, cll)
tf = isfield(cc, 'beta_mode') && isfield(cn, 'beta_mode') && isfield(cll, 'beta_mode') && ...
    string(cc.beta_mode) == "absolute" && string(cn.beta_mode) == "absolute" && string(cll.beta_mode) == "absolute";
end
