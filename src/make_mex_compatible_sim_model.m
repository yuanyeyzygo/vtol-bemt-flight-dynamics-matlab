function sim_model = make_mex_compatible_sim_model(sim_model, keep_dynamic_derivatives)
%MAKE_MEX_COMPATIBLE_SIM_MODEL Remove run-time-only handles from sim_model.
%
% MATLAB Coder compiles sim_model as a constant argument for the fast MEX
% response step. Function handles and unused lookup objects are therefore
% removed here so the MEX uses the numeric default/disk-model path.

if nargin < 2
    keep_dynamic_derivatives = false;
end

remove_fields = {'force_fun', 'fuselage_fun', ...
    'cd', 'cl', 'cm', 'cc', 'cn', 'cll', 'elev', 'rudd', 'airp', ...
    'flap_model', 'aircraft_integrator', ...
    'disk_flap_state_update', 'disk_state_integrator'};
sim_model = remove_existing_fields(sim_model, remove_fields);

if isfield(sim_model, 'rotor_profile')
    sim_model.rotor_profile = remove_existing_fields(sim_model.rotor_profile, ...
        {'cl_lookup', 'cd_lookup'});
end

if isfield(sim_model, 'rotor_bemt_options')
    sim_model.rotor_bemt_options = remove_existing_fields(sim_model.rotor_bemt_options, ...
        {'flap_model', 'flap_integrator', 'inflow_model', 'inflow_tau_mode', ...
        'disk_state_integrator'});
end

if ~keep_dynamic_derivatives && isfield(sim_model, 'fuselage_geometry') && ...
        isfield(sim_model.fuselage_geometry, 'dynamic_derivatives')
    sim_model.fuselage_geometry = rmfield(sim_model.fuselage_geometry, 'dynamic_derivatives');
end
end

function s = remove_existing_fields(s, names)
for ii = 1:numel(names)
    if isfield(s, names{ii})
        s = rmfield(s, names{ii});
    end
end
end
