function [rho, temperature_k, pressure_pa, speed_of_sound_mps] = isa_atmosphere(altitude_m)
%ISA_ATMOSPHERE International Standard Atmosphere density model.
%
% altitude_m is geometric altitude above mean sea level in meters. The model
% covers the troposphere and the first isothermal stratosphere layer, which is
% enough for the low-altitude VTOL response cases here.

h = altitude_m;
if ~isfinite(h)
    h = 0;
end

T0 = 288.15;
p0 = 101325.0;
g0 = 9.80665;
R_air = 287.05287;
gamma_air = 1.4;
lapse = -0.0065;
h_tropopause = 11000.0;

if h <= h_tropopause
    temperature_k = T0 + lapse * h;
    temperature_k = max(temperature_k, 1.0);
    pressure_pa = p0 * (temperature_k / T0)^(-g0 / (lapse * R_air));
else
    T11 = T0 + lapse * h_tropopause;
    p11 = p0 * (T11 / T0)^(-g0 / (lapse * R_air));
    temperature_k = T11;
    pressure_pa = p11 * exp(-g0 * (h - h_tropopause) / (R_air * T11));
end

rho = pressure_pa / (R_air * temperature_k);
speed_of_sound_mps = sqrt(gamma_air * R_air * temperature_k);
end
