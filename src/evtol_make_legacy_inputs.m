function inputs = evtol_make_legacy_inputs(cfg, model)
%EVTOL_MAKE_LEGACY_INPUTS Adapt the new config/model structs to legacy names.
%   The legacy BEMTFLAP function currently accepts six CL/CD section handles
%   and is still written around six rotors. This adapter makes that boundary
%   explicit while the solver internals are being loop-refactored.

arguments
    cfg struct
    model struct
end

if cfg.rotors.count ~= 6
    warning("Legacy BEMTFLAP_legacy.m still assumes 6 rotors. Use cfg.rotors.count = 6 until the trim loop is refactored.");
end
if cfg.rotors.blade_count > 5
    warning("Legacy flapping state vector stores 5 blade beta states. Refactor bemt_flapp before using more than 5 blades.");
end

inputs = struct();
inputs.R = cfg.rotors.radius_m;
inputs.Nb = cfg.rotors.blade_count;
inputs.omega = cfg.rotors.omega_rad_s;
inputs.I_beta = cfg.rotors.I_beta;
inputs.k_beta = cfg.rotors.k_beta;
inputs.F = model.chord.fun;
inputs.pre_twist = model.pretwist.fun;
inputs.rotor_positions_m = model.rotor_positions_m;
inputs.rotational_direction = cfg.rotors.rotational_direction(:).';

for k = 1:6
    idx = min(k, numel(model.airfoil.cl));
    inputs.cl{k} = model.airfoil.cl{idx};
    inputs.cd{k} = model.airfoil.cd{idx};
end

inputs.fuselage = model.fuselage;
inputs.controls = model.controls;
end
