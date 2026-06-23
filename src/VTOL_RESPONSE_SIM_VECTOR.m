function y = VTOL_RESPONSE_SIM_VECTOR(u)
%VTOL_RESPONSE_SIM_VECTOR Vector I/O wrapper for Simulink interpreted block.
%
% Input vector layout:
%   [t; x(12); rotor_state; control_delta(4); rotor_tilt_angle_deg(6); fixed_control_delta(3); dt]
%
% Output vector layout:
%   [x_next(12); rotor_state_next; forces(6); vi(6); vi_qs(6); tau_s(6)]

sim_model = evalin('base', 'sim_model');
n_rotor_state = sim_model.n_rotors * sim_model.rotor_state_size;

u = u(:);
expected_n = 1 + 12 + n_rotor_state + 4 + 6 + 3 + 1;
if numel(u) ~= expected_n
    error('VTOL_RESPONSE_SIM_VECTOR:BadInputSize', ...
        'Expected input vector length %d, got %d.', expected_n, numel(u));
end

pos = 1;
t = u(pos); pos = pos + 1;
x = u(pos:pos+11); pos = pos + 12;
rotor_state = u(pos:pos+n_rotor_state-1); pos = pos + n_rotor_state;
control_delta = u(pos:pos+3); pos = pos + 4;
rotor_tilt_angle_deg = u(pos:pos+5); pos = pos + 6;
fixed_control_delta = u(pos:pos+2); pos = pos + 3;
dt = u(pos);

[x_next, rotor_state_next, forces, vi, vi_qs, tau_s] = VTOL_RESPONSE_SIM_STEP( ...
    t, x, rotor_state, control_delta, rotor_tilt_angle_deg, fixed_control_delta, dt, sim_model);

y = [x_next(:); rotor_state_next(:); forces(:); vi(:); vi_qs(:); tau_s(:)];
end
