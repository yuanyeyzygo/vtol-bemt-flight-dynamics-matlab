function trim_results = RUN_TRIM_AND_STABILITY(state, vehicle, cfg, model, trim_options)
%RUN_TRIM_AND_STABILITY Run trim, stability, and control-derivative analysis.
%   This public entry point uses the clean RUN_ME configuration and the
%   current rotor/airframe model switches. It does not require the original
%   legacy script or private lookup data when RUN_ME is in the default
%   analytic profile.
%
%   Direct use:
%       trim_results = RUN_TRIM_AND_STABILITY
%
%   From RUN_ME, the configured state/vehicle/cfg/model structs are passed in
%   directly so edited user settings are respected.

root = fileparts(mfilename("fullpath"));
addpath(fullfile(root, "src"));

if nargin < 5
    [state, vehicle, cfg, model, ~, options] = RUN_ME("config_only", "default");
    trim_options = options.trim;
end

trim_results = evtol_run_trim_stability(state, vehicle, cfg, model, trim_options);

fprintf("\n=== Trim / Stability Results ===\n");
fprintf("Speed points completed: %d\n", size(trim_results.MMA, 1) / 9);
fprintf("Trim table columns:\n");
disp(trim_results.trim_table_columns);
disp(trim_results.Mttt);

fprintf("Last speed-point A matrix size: %d x %d\n", size(trim_results.A, 1), size(trim_results.A, 2));
fprintf("Last speed-point B_all matrix size: %d x %d\n", size(trim_results.B_all, 1), size(trim_results.B_all, 2));
fprintf("State order:\n");
disp(trim_results.state_names);
fprintf("Control order:\n");
disp(trim_results.control_names);
fprintf("Eigenvalues of final A:\n");
disp(trim_results.eig_A);

assignin("base", "trim_results", trim_results);
end
