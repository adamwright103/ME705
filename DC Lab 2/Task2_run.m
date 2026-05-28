% Pole Placement for PI Controller - Sensitivity Analysis Included
clear; clc; close all;
set(0, 'DefaultFigureWindowStyle', 'docked');

% 1. Define Plant Parameters from P(z) and get continuous model
num_P = 0.0162;
den_P_root = 0.759;
[num_Hs, den_Hs] = setupGantryModel(1.5); % Initialize with nominal 1.5kg

% 2. Define Desired Closed-Loop Poles
p1 = 0.57; 
p2 = 0.57;
velocityStep = 0.7; % Nominal speed setpoint
noise_power = 0;

% 3. Calculate Controller Coefficients (q0, q1) - FIXED NOMINAL CONTROLLER
q0 = (1.759 - (p1 + p2)) / num_P;
q1 = ((p1 * p2) - den_P_root) / num_P;

% 4. Format for Simulink Workspace
num_C = [q0, q1]; 
den_C = [1, -1];  

%% --- Execution & Analysis (Baseline Nominal Run) ---

out = sim('MotorSlider_Sim_controlled.slx');

time = out.VelocityDB.Time;
vel_DB = out.VelocityDB.Data;
vel_P = out.VelocityP.Data;

ref_signal = zeros(size(time));
ref_signal(time >= 0.3) = velocityStep;

% --- Figure 1: Baseline Comparison ---
figure('Name', 'Controller Comparison (Nominal)');
subplot(2,1,1);
plot(time, ref_signal, 'k--', 'LineWidth', 1.5); hold on;
plot(time, vel_DB, 'b-', 'LineWidth', 1.5);
title('Deadbeat Controller Output vs Reference (Nominal)');
xlabel('Time (s)'); ylabel('Velocity'); legend('Reference', 'Deadbeat', 'Location', 'best'); grid on;

subplot(2,1,2);
plot(time, ref_signal, 'k--', 'LineWidth', 1.5); hold on;
plot(time, vel_P, 'r-', 'LineWidth', 1.5);
title('Pole Placement Controller Output vs Reference (Nominal)');
xlabel('Time (s)'); ylabel('Velocity'); legend('Reference', 'Pole Placement', 'Location', 'best'); grid on;

% Baseline performance calculations
step_idx = find(time >= 0.3, 1);
t_step = time(step_idx:end) - 0.3;
[rt_P_nom, os_P_nom, st_P_nom] = calculate_step_metrics(t_step, vel_P(step_idx:end), velocityStep);


%% --- AUTOMATED SENSITIVITY TESTING ---

% Define Test Vectors
numVariations = 20;
mass_vec = linspace(0.5,2,numVariations);
speed_vec = linspace(0.1,1.3,numVariations);

% Preallocate Data Arrays for Pole Placement Controller
rt_mass_results = zeros(size(mass_vec));
os_mass_results = zeros(size(mass_vec));
st_mass_results = zeros(size(mass_vec));

rt_speed_results = zeros(size(speed_vec));
os_speed_results = zeros(size(speed_vec));
st_speed_results = zeros(size(speed_vec));

% --- Test 1: Mass Uncertainty Loop (Speed fixed at nominal 0.7) ---
velocityStep = 0.7; 
for i = 1:length(mass_vec)
    [num_Hs, den_Hs] = setupGantryModel(mass_vec(i)); % Update physical plant mass
    
    out_sim = sim('MotorSlider_Sim_controlled.slx');
    t_sim = out_sim.VelocityP.Time;
    v_sim = out_sim.VelocityP.Data;
    
    s_idx = find(t_sim >= 0.3, 1);
    [rt_mass_results(i), os_mass_results(i), st_mass_results(i)] = ...
        calculate_step_metrics(t_sim(s_idx:end)-0.3, v_sim(s_idx:end), velocityStep);
end

% --- Test 2: Speed Setpoint Loop (Mass fixed at nominal 1.5kg) ---
[num_Hs, den_Hs] = setupGantryModel(1.5); % Reset plant to nominal mass
for j = 1:length(speed_vec)
    velocityStep = speed_vec(j); % Update step input magnitude
    
    out_sim = sim('MotorSlider_Sim_controlled.slx');
    t_sim = out_sim.VelocityP.Time;
    v_sim = out_sim.VelocityP.Data;
    
    s_idx = find(t_sim >= 0.3, 1);
    [rt_speed_results(j), os_speed_results(j), st_speed_results(j)] = ...
        calculate_step_metrics(t_sim(s_idx:end)-0.3, v_sim(s_idx:end), velocityStep);
end


%% --- PRINT PRINT PRINT (Command Window Tables) ---

fprintf('\n==================================================\n');
fprintf('        CONTROLLER SENSITIVITY METRICS DATA       \n');
fprintf('==================================================\n\n');

Mass_Sensitivity_Table = table(mass_vec', rt_mass_results', os_mass_results', st_mass_results', ...
    'VariableNames', {'Mass_kg', 'RiseTime_s', 'Overshoot_Percent', 'SettlingTime_s'});
disp('--- Sensitivity to Plant Mass Variations (Speed = 0.7) ---');
disp(Mass_Sensitivity_Table);

Speed_Sensitivity_Table = table(speed_vec', rt_speed_results', os_speed_results', st_speed_results', ...
    'VariableNames', {'Setpoint_Speed', 'RiseTime_s', 'Overshoot_Percent', 'SettlingTime_s'});
disp('--- Sensitivity to Input Speed Setpoints (Mass = 1.5kg) ---');
disp(Speed_Sensitivity_Table);


%% --- PLOT PLOT PLOT (Figure 2: 1 Row, 2 Subplots) ---

figure('Name', 'Controller Sensitivity Analysis Trends');

% Subplot 1: Mass Changing Trends
subplot(1, 2, 1);
plot(mass_vec, rt_mass_results ./ rt_P_nom, '-ob', 'LineWidth', 1.5); hold on;
plot(mass_vec, (os_mass_results + 1e-3) ./ (os_P_nom + 1e-3), '-or', 'LineWidth', 1.5); % offset protects against 0/0
plot(mass_vec, st_mass_results ./ st_P_nom, '-ok', 'LineWidth', 1.5);
yline(1.0, 'k--', 'Nominal Baseline', 'LabelHorizontalAlignment', 'left');
title('Sensitivity to Carriage Mass Changes');
xlabel('Actual Carriage Mass (kg)');
ylabel('Normalized Performance Factor (Value / Nominal)');
legend('Rise Time', 'Overshoot', 'Settling Time', 'Location', 'best');
set(gca, 'YScale', 'log');
grid on;

% Subplot 2: Speed Changing Trends
subplot(1, 2, 2);
plot(speed_vec, rt_speed_results ./ rt_P_nom, '-ob', 'LineWidth', 1.5); hold on;
plot(speed_vec, (os_speed_results + 1e-3) ./ (os_P_nom + 1e-3), '-or', 'LineWidth', 1.5);
plot(speed_vec, st_speed_results ./ st_P_nom, '-ok', 'LineWidth', 1.5);
yline(1.0, 'k--', 'Nominal Baseline', 'LabelHorizontalAlignment', 'left');
title('Sensitivity to Velocity Setpoint Changes');
xlabel('Speed Setpoint Value');
ylabel('Normalized Performance Factor (Value / Nominal)');
legend('Rise Time', 'Overshoot', 'Settling Time', 'Location', 'best');
set(gca, 'YScale', 'log');
grid on;


%% --- Local Functions ---
function [rise_time, overshoot, settling_time] = calculate_step_metrics(t, y, ref)
    % 1. Overshoot %
    peak_val = max(y);
    overshoot = max(0, ((peak_val - ref) / ref) * 100); 

    % 2. 90% Rise Time
    idx_90 = find(y >= 0.9 * ref, 1);
    if ~isempty(idx_90)
        rise_time = t(idx_90);
    else
        rise_time = NaN; 
    end

    % 3. Settling Time (2%)
    upper_bound = ref * 1.02;
    lower_bound = ref * 0.98;
    out_of_bounds_idx = find(y > upper_bound | y < lower_bound, 1, 'last');
    
    if isempty(out_of_bounds_idx)
        settling_time = 0; 
    elseif out_of_bounds_idx == length(y)
        settling_time = NaN; 
    else
        settling_time = t(out_of_bounds_idx + 1); 
    end
end
