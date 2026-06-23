function BUILD_FAST_DISK_MEX()
%BUILD_FAST_DISK_MEX Rebuild fast disk MEX through the normal setup path.
%
% The normal workflow is:
%   1. Edit RUN_RESPONSE_SIMULINK_SETUP.m.
%   2. Run RUN_RESPONSE_SIMULINK_SETUP.
%   3. Run Simulink or RUN_FAST_DISK_MEX_RESPONSE.
%
% This wrapper is kept for convenience and simply forces disk mode.

flap_model_override = "disk"; %#ok<NASGU>
RUN_RESPONSE_SIMULINK_SETUP;
end
