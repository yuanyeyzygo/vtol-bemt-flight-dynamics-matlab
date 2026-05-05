function cfg = evtol_default_config(model_profile)
%EVTOL_DEFAULT_CONFIG Return the configuration produced by RUN_ME.
%   RUN_ME.m is the single user-editable interface. This function is kept for
%   examples and older helper scripts that expect a cfg struct.

if nargin < 1
    model_profile = "default";
end

root = fileparts(fileparts(mfilename("fullpath")));
run_me_file = fullfile(root, "RUN_ME.m");
if exist(run_me_file, "file") ~= 2
    error("RUN_ME.m not found at expected project root: %s", root);
end

old_path = path;
cleanup_path = onCleanup(@() path(old_path)); %#ok<NASGU>
addpath(root);
addpath(fullfile(root, "src"));

[~, ~, cfg] = RUN_ME("config_only", model_profile);
end
