function delta = fuselage_dynamic_derivative_deltas(U, pqr, velocity, C_a, b, dyn)
%FUSELAGE_DYNAMIC_DERIVATIVE_DELTAS Dynamic aero coefficient increments.
%
% Returns [dCD dCL dCm dCC dCn dCl]. The derivatives are nondimensionalized
% using q*C_a/(2V), p*b/(2V), r*b/(2V), alpha_dot*C_a/(2V),
% and beta_dot*b/(2V).

    delta = zeros(1,6);
    if nargin < 6 || isempty(dyn) || ~isstruct(dyn)
        return;
    end
    if isfield(dyn, 'enabled') && ~logical(dyn.enabled)
        return;
    end

    min_velocity = 1e-6;
    if isfield(dyn, 'min_velocity_mps')
        min_velocity = dyn.min_velocity_mps;
    end
    if ~isfinite(velocity) || velocity < min_velocity
        return;
    end

    p = pqr(1);
    q = pqr(2);
    r = pqr(3);

    p_hat = p * b / (2 * velocity);
    q_hat = q * C_a / (2 * velocity);
    r_hat = r * b / (2 * velocity);

    alpha_dot = 0;
    beta_dot = 0;
    mode_id = 1;
    if isfield(dyn, 'alpha_beta_dot_mode_id')
        mode_id = dyn.alpha_beta_dot_mode_id;
    end

    if mode_id == 1
        U = U(:);
        pqr = pqr(:);
        U_dot = -cross(pqr, U);

        alpha_den = U(1)^2 + U(3)^2;
        if alpha_den > min_velocity^2
            alpha_dot = (U(1)*U_dot(3) - U(3)*U_dot(1)) / alpha_den;
        end

        beta_den = U(1)^2 + U(2)^2;
        if beta_den > min_velocity^2
            beta_dot = (U(1)*U_dot(2) - U(2)*U_dot(1)) / beta_den;
        end
    elseif mode_id ~= 0
        error('FuselageDynamicDerivatives:BadMode', ...
            'alpha_beta_dot_mode_id must be 0 (zero) or 1 (kinematic).');
    end

    alpha_dot_hat = alpha_dot * C_a / (2 * velocity);
    beta_dot_hat = beta_dot * b / (2 * velocity);

    delta(2) = get_dyn_field(dyn, 'CLq') * q_hat;
    delta(3) = get_dyn_field(dyn, 'Cmq') * q_hat + ...
               get_dyn_field(dyn, 'Cma_dot') * alpha_dot_hat;
    delta(4) = get_dyn_field(dyn, 'Cyp') * p_hat + ...
               get_dyn_field(dyn, 'Cyr') * r_hat;
    delta(5) = get_dyn_field(dyn, 'Cnp') * p_hat + ...
               get_dyn_field(dyn, 'Cnr') * r_hat + ...
               get_dyn_field(dyn, 'Cn_beta_dot') * beta_dot_hat;
    delta(6) = get_dyn_field(dyn, 'Clp') * p_hat + ...
               get_dyn_field(dyn, 'Clr') * r_hat;
end

function value = get_dyn_field(dyn, name)
    value = 0;
    if isfield(dyn, name)
        value = dyn.(name);
    end
end
