function positions_m = evtol_rotor_locations(cfg, tilt_angle_deg)
%EVTOL_ROTOR_LOCATIONS Return rotor hub positions in aircraft/body axes.
%   Manual mode uses cfg.rotors.positions_m directly. Default mode preserves
%   the six-rotor layout from the legacy script, with simple tilt dependence.

arguments
    cfg struct
    tilt_angle_deg double = cfg.trim.tilt_angle_deg
end

mode = lower(string(cfg.rotors.position_mode));
switch mode
    case "manual"
        positions_m = cfg.rotors.positions_m;
    case "default"
        tilt = deg2rad(tilt_angle_deg);
        positions_mm = [ ...
            2377 + 700 * sin(tilt),  6000, -2000 + cos(tilt); ...
            1900,                    2500, -1850; ...
            1900,                   -2500, -1850; ...
            2377 + 700 * sin(tilt), -6000, -2000 + cos(tilt); ...
            4754 + 700 * sin(tilt),  2500, -3100 + cos(tilt); ...
            4754 + 700 * sin(tilt), -2500, -3100 + cos(tilt)];
        positions_m = positions_mm ./ 1000;
    otherwise
        error('cfg.rotors.position_mode must be "manual" or "default".');
end

if size(positions_m, 1) ~= cfg.rotors.count || size(positions_m, 2) ~= 3
    error("cfg.rotors.positions_m must be cfg.rotors.count x 3.");
end
end
