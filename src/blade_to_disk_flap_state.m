function disk_state = blade_to_disk_flap_state(blade_state, Nb, omega, rotational_direction)
%BLADE_TO_DISK_FLAP_STATE Project blade flap states onto 0/1 harmonic disk states.
%
% blade_state = [beta(1:Nb); dbeta(1:Nb); vi]
% disk_state  = [beta0; beta1c; beta1s; dbeta0; dbeta1c; dbeta1s; vi]

blade_state = blade_state(:);
if numel(blade_state) ~= 2*Nb + 1
    error('BEMTFLAP:BadBladeState', ...
        'blade_state must contain 2*Nb+1 values.');
end

psi = rotational_direction * (0:Nb-1).' * 2*pi/Nb;
A = [ones(Nb,1), cos(psi), sin(psi)];

beta_blade = blade_state(1:Nb);
dbeta_blade = blade_state(Nb+1:2*Nb);
vi = blade_state(end);

q = A \ beta_blade;

omega_signed = omega * rotational_direction;
kinematic_rate = omega_signed * (-q(2)*sin(psi) + q(3)*cos(psi));
qdot = A \ (dbeta_blade - kinematic_rate);

disk_state = [q(:); qdot(:); vi];
end
