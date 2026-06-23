function [x_next, rotor_state_next, forces, vi, vi_qs, tau_s] = VTOL_RESPONSE_SIM_STEP(t, x, rotor_state, control_delta, rotor_tilt_angle_deg, fixed_control_delta, dt, sim_model)
%VTOL_RESPONSE_SIM_STEP Discrete nonlinear response step for Simulink.
%
% This wrapper advances the aircraft state over one Simulink sample time and
% then advances the rotor periodic states / uniform induced velocity once.

integrator = "rk4";
if isfield(sim_model, 'aircraft_integrator')
    integrator = lower(string(sim_model.aircraft_integrator));
end

switch integrator
    case "euler"
        [xdot, ~] = VTOL_RESPONSE_STEP(t, x, rotor_state, control_delta, fixed_control_delta, rotor_tilt_angle_deg, 0, sim_model);
        x_next = x(:) + dt * xdot(:);

    case "rk4"
        [k1, ~] = VTOL_RESPONSE_STEP(t, x, rotor_state, control_delta, fixed_control_delta, rotor_tilt_angle_deg, 0, sim_model);
        [k2, ~] = VTOL_RESPONSE_STEP(t + 0.5*dt, x(:) + 0.5*dt*k1(:), rotor_state, control_delta, fixed_control_delta, rotor_tilt_angle_deg, 0, sim_model);
        [k3, ~] = VTOL_RESPONSE_STEP(t + 0.5*dt, x(:) + 0.5*dt*k2(:), rotor_state, control_delta, fixed_control_delta, rotor_tilt_angle_deg, 0, sim_model);
        [k4, ~] = VTOL_RESPONSE_STEP(t + dt, x(:) + dt*k3(:), rotor_state, control_delta, fixed_control_delta, rotor_tilt_angle_deg, 0, sim_model);
        x_next = x(:) + (dt/6) * (k1(:) + 2*k2(:) + 2*k3(:) + k4(:));

    otherwise
        error('VTOL_RESPONSE_SIM_STEP:BadIntegrator', ...
            'sim_model.aircraft_integrator must be "euler" or "rk4".');
end

[~, rotor_state_next, forces, vi, vi_qs, tau_s] = VTOL_RESPONSE_STEP( ...
    t + dt, x_next, rotor_state, control_delta, fixed_control_delta, rotor_tilt_angle_deg, dt, sim_model);
end
