function [num_H, den_H] = setupGantryModel(m)
% Calculates the continuous transfer function H(s) of the DC Motor Gantry.
% Input: m (mass in kg). Defaults to 1.5 kg if omitted.
% Outputs: num_H, den_H (normalized transfer function coefficients)

if nargin < 1
    m = 1.5; % Default mass
end

% --- Constants from Datasheet / Derivation ---
r_pulley = 0.005; % m
GR = 3;           % Gear Ratio
J_rotor = 1.5e-6; % kg.m^2 
k_m = 0.0242;     % Motor constant (kt = kb = km)
R_w = 7.704;      % Winding Resistance (Ohms)
B = 2.04e-6;      % Viscous friction coefficient

% --- Derived Inertia ---
% J = J_rotor + (m * r_pulley^2) / GR^2
J = J_rotor + (m * r_pulley^2) / (GR^2);

% --- Transfer Function Construction ---
% H(s) = (r * km) / (GR * (Rw * (Js + B) + km^2))

% Numerator term
num_raw = r_pulley * k_m;

% Denominator terms: s^1 and s^0
den_s1_raw = GR * R_w * J;
den_s0_raw = GR * (R_w * B + k_m^2);

% Normalize to standard form (leading denominator coefficient = 1)
num_H = num_raw / den_s1_raw;
den_H = [1, (den_s0_raw / den_s1_raw)];
end