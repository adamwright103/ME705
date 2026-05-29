% Pole Placement for PI Controller - Sensitivity Analysis Included
clear; clc; close all;
set(0, 'DefaultFigureWindowStyle', 'docked');

% 1. Define Plant Parameters from P(z) and get continuous model
num_P = 0.0162;
den_P_root = 0.759;
[num_Hs, den_Hs] = setupGantryModel(1.5); % Initialize with nominal 1.5kg

% 2. Define Desired Closed-Loop Poles
p1 = 0.59; 
p2 = 0.59;
velocityStep = 0.7; % Nominal speed setpoint
noise_power = 0;    % Nominal noise

% 3. Calculate Controller Coefficients (q0, q1) - FIXED NOMINAL CONTROLLER
q0 = (1.759 - (p1 + p2)) / num_P;
q1 = ((p1 * p2) - den_P_root) / num_P;

% 4. Format for Simulink Workspace
num_C = [q0, q1]; 
den_C = [1, -1];  

% 5. FORCE FIXED TIME STEP (0.0001s) ON BOTH SIMULINK MODELS
% This modifies the configuration in memory without altering your hard files.
models = {'MotorSlider_Sim_controlled', 'MotorSlider_RS_controlled'};
for m = 1:length(models)
    load_system(models{m}); 
    set_param(models{m}, 'SolverType', 'Fixed-step');
    set_param(models{m}, 'FixedStep', '0.0001');
end

%% --- Execution & Analysis (Baseline Nominal Run) ---

% Simulate Simplified Model
out_Sim = sim('MotorSlider_Sim_controlled');
time_Sim = out_Sim.VelocityDB.Time;
vel_DB_Sim = out_Sim.VelocityDB.Data;
vel_P_Sim = out_Sim.VelocityP.Data;

% Simulate Realistic Model
out_RS = sim('MotorSlider_RS_controlled');
time_RS = out_RS.VelocityDB.Time;
vel_DB_RS = out_RS.VelocityDB.Data;
vel_P_RS = out_RS.VelocityP.Data;

% Generate Reference Signal (using time from simplified model for plotting)
ref_signal = zeros(size(time_Sim));
ref_signal(time_Sim >= 0.3) = velocityStep;

% --- Figure 1: Baseline Comparison (Simple vs Realistic) ---
figure('Name', 'Controller Comparison (Nominal: Simple vs Realistic)');

% Deadbeat Comparison Subplot
subplot(2,1,1);
plot(time_Sim, ref_signal, 'k--', 'LineWidth', 1.5); hold on;
plot(time_Sim, vel_DB_Sim, 'b-', 'LineWidth', 1.5);
plot(time_RS, vel_DB_RS, 'r--', 'LineWidth', 1.5);
title('Deadbeat Controller Output: Simplified vs. Realistic Plant');
xlabel('Time (s)'); ylabel('Velocity'); 
legend('Reference', 'Deadbeat (Simple Plant)', 'Deadbeat (Realistic Plant)', 'Location', 'best'); 
grid on;

% Pole Placement Comparison Subplot
subplot(2,1,2);
plot(time_Sim, ref_signal, 'k--', 'LineWidth', 1.5); hold on;
plot(time_Sim, vel_P_Sim, 'b-', 'LineWidth', 1.5);
plot(time_RS, vel_P_RS, 'r--', 'LineWidth', 1.5);
title('Pole Placement Controller Output: Simplified vs. Realistic Plant');
xlabel('Time (s)'); ylabel('Velocity'); 
legend('Reference', 'Pole Placement (Simple Plant)', 'Pole Placement (Realistic Plant)', 'Location', 'best'); 
grid on;


%% --- AUTOMATED SENSITIVITY TESTING ---
% (Testing only on the simplified model as per design requirements)

% Define Test Vectors
numVariations = 5;
mass_vec = linspace(0.5, 2.5, numVariations);
speed_vec = linspace(0.2, 1, numVariations);
noise_vec = [0, 1e-9, 1e-8, 1e-7, 1e-6]; % Log scale range for noise

% --- Test 1: Mass Uncertainty Loop ---
velocityStep = 0.7; 
noise_power = 0;
for i = 1:length(mass_vec)
    [num_Hs, den_Hs] = setupGantryModel(mass_vec(i)); 
    out_sim = sim('MotorSlider_Sim_controlled');
    
    t_sim = out_sim.VelocityP.Time;
    v_sim_P = out_sim.VelocityP.Data;
    v_sim_DB = out_sim.VelocityDB.Data;
    
    % Store for plotting
    hist_mass_t{i} = t_sim; 
    hist_mass_P{i} = v_sim_P; 
    hist_mass_DB{i} = v_sim_DB;
    
    s_idx = find(t_sim >= 0.3, 1);
    [rt_mass_P(i,1), os_mass_P(i,1), st_mass_P(i,1)] = calculate_step_metrics(t_sim(s_idx:end)-0.3, v_sim_P(s_idx:end), velocityStep);
    [rt_mass_DB(i,1), os_mass_DB(i,1), st_mass_DB(i,1)] = calculate_step_metrics(t_sim(s_idx:end)-0.3, v_sim_DB(s_idx:end), velocityStep);
end

% --- Test 2: Speed Setpoint Loop ---
[num_Hs, den_Hs] = setupGantryModel(1.5); % Reset plant
noise_power = 0;
for j = 1:length(speed_vec)
    velocityStep = speed_vec(j); 
    out_sim = sim('MotorSlider_Sim_controlled');
    
    t_sim = out_sim.VelocityP.Time;
    v_sim_P = out_sim.VelocityP.Data;
    v_sim_DB = out_sim.VelocityDB.Data;
    
    % Store for plotting
    hist_speed_t{j} = t_sim; 
    hist_speed_P{j} = v_sim_P; 
    hist_speed_DB{j} = v_sim_DB;
    
    s_idx = find(t_sim >= 0.3, 1);
    [rt_speed_P(j,1), os_speed_P(j,1), st_speed_P(j,1)] = calculate_step_metrics(t_sim(s_idx:end)-0.3, v_sim_P(s_idx:end), velocityStep);
    [rt_speed_DB(j,1), os_speed_DB(j,1), st_speed_DB(j,1)] = calculate_step_metrics(t_sim(s_idx:end)-0.3, v_sim_DB(s_idx:end), velocityStep);
end

% --- Test 3: Noise Power Loop ---
[num_Hs, den_Hs] = setupGantryModel(1.5); % Reset plant
velocityStep = 0.7;                       % Reset speed
for k = 1:length(noise_vec)
    noise_power = noise_vec(k);
    out_sim = sim('MotorSlider_Sim_controlled');
    
    t_sim = out_sim.VelocityP.Time;
    v_sim_P = out_sim.VelocityP.Data;
    v_sim_DB = out_sim.VelocityDB.Data;
    
    % Store for plotting
    hist_noise_t{k} = t_sim; 
    hist_noise_P{k} = v_sim_P; 
    hist_noise_DB{k} = v_sim_DB;
    
    s_idx = find(t_sim >= 0.3, 1);
    [rt_noise_P(k,1), os_noise_P(k,1), st_noise_P(k,1)] = calculate_step_metrics(t_sim(s_idx:end)-0.3, v_sim_P(s_idx:end), velocityStep);
    [rt_noise_DB(k,1), os_noise_DB(k,1), st_noise_DB(k,1)] = calculate_step_metrics(t_sim(s_idx:end)-0.3, v_sim_DB(s_idx:end), velocityStep);
end

%% --- PRINT PRINT PRINT (Command Window Tables) ---
fprintf('\n==================================================\n');
fprintf('        CONTROLLER SENSITIVITY METRICS DATA       \n');
fprintf('==================================================\n\n');

% Mass Tables
disp('--- DEADBEAT: Sensitivity to Plant Mass Variations (Speed = 0.7) ---');
disp(table(mass_vec', rt_mass_DB, os_mass_DB, st_mass_DB, 'VariableNames', {'Mass_kg', 'RiseTime_s', 'Overshoot_Percent', 'SettlingTime_s'}));
disp('--- POLE PLACEMENT: Sensitivity to Plant Mass Variations (Speed = 0.7) ---');
disp(table(mass_vec', rt_mass_P, os_mass_P, st_mass_P, 'VariableNames', {'Mass_kg', 'RiseTime_s', 'Overshoot_Percent', 'SettlingTime_s'}));

% Speed Tables
disp('--- DEADBEAT: Sensitivity to Input Speed Setpoints (Mass = 1.5kg) ---');
disp(table(speed_vec', rt_speed_DB, os_speed_DB, st_speed_DB, 'VariableNames', {'Setpoint_Speed', 'RiseTime_s', 'Overshoot_Percent', 'SettlingTime_s'}));
disp('--- POLE PLACEMENT: Sensitivity to Input Speed Setpoints (Mass = 1.5kg) ---');
disp(table(speed_vec', rt_speed_P, os_speed_P, st_speed_P, 'VariableNames', {'Setpoint_Speed', 'RiseTime_s', 'Overshoot_Percent', 'SettlingTime_s'}));

% Noise Tables
disp('--- DEADBEAT: Sensitivity to Noise Power (Mass = 1.5kg, Speed = 0.7) ---');
disp(table(noise_vec', rt_noise_DB, os_noise_DB, st_noise_DB, 'VariableNames', {'Noise_Power', 'RiseTime_s', 'Overshoot_Percent', 'SettlingTime_s'}));
disp('--- POLE PLACEMENT: Sensitivity to Noise Power (Mass = 1.5kg, Speed = 0.7) ---');
disp(table(noise_vec', rt_noise_P, os_noise_P, st_noise_P, 'VariableNames', {'Noise_Power', 'RiseTime_s', 'Overshoot_Percent', 'SettlingTime_s'}));

%% --- PLOT PLOT PLOT (Figure 2: Overlay Step Responses) ---
figure('Name', 'Controller Sensitivity Analysis: Step Responses');
colors = lines(max([length(mass_vec), length(speed_vec), length(noise_vec)]));

% Define common max time for reference lines based on simulation time
t_max = max(hist_mass_t{1});

% Row 1: Deadbeat
% 1.1 DB Mass
subplot(2, 3, 1); hold on; grid on;
plot([0, 0.3, 0.3, t_max], [0, 0, 0.7, 0.7], 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
for i = 1:length(mass_vec)
    plot(hist_mass_t{i}, hist_mass_DB{i}, 'Color', colors(i,:), 'DisplayName', sprintf('Mass = %.2f', mass_vec(i)));
end
title('Deadbeat: Mass Variation'); xlabel('Time (s)'); ylabel('Velocity'); legend('Location','best');

% 1.2 DB Speed (Normalized)
subplot(2, 3, 2); hold on; grid on;
plot([0, 0.3, 0.3, t_max], [0, 0, 1, 1], 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
for j = 1:length(speed_vec)
    plot(hist_speed_t{j}, hist_speed_DB{j} / speed_vec(j), 'Color', colors(j,:), 'DisplayName', sprintf('Speed = %.2f', speed_vec(j)));
end
title('Deadbeat: Speed Variation (Normalized)'); xlabel('Time (s)'); ylabel('Norm. Velocity'); legend('Location','best');

% 1.3 DB Noise
subplot(2, 3, 3); hold on; grid on;
plot([0, 0.3, 0.3, t_max], [0, 0, 0.7, 0.7], 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
for k = 1:length(noise_vec)
    plot(hist_noise_t{k}, hist_noise_DB{k}, 'Color', colors(k,:), 'DisplayName', sprintf('Noise = %g', noise_vec(k)));
end
title('Deadbeat: Noise Variation'); xlabel('Time (s)'); ylabel('Velocity'); legend('Location','best');

% Row 2: Pole Placement
% 2.1 PP Mass
subplot(2, 3, 4); hold on; grid on;
plot([0, 0.3, 0.3, t_max], [0, 0, 0.7, 0.7], 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
for i = 1:length(mass_vec)
    plot(hist_mass_t{i}, hist_mass_P{i}, 'Color', colors(i,:), 'DisplayName', sprintf('Mass = %.2f', mass_vec(i)));
end
title('Pole Placement: Mass Variation'); xlabel('Time (s)'); ylabel('Velocity'); legend('Location','best');

% 2.2 PP Speed (Normalized)
subplot(2, 3, 5); hold on; grid on;
plot([0, 0.3, 0.3, t_max], [0, 0, 1, 1], 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
for j = 1:length(speed_vec)
    plot(hist_speed_t{j}, hist_speed_P{j} / speed_vec(j), 'Color', colors(j,:), 'DisplayName', sprintf('Speed = %.2f', speed_vec(j)));
end
title('Pole Placement: Speed Var (Normalized)'); xlabel('Time (s)'); ylabel('Norm. Velocity'); legend('Location','best');

% 2.3 PP Noise
subplot(2, 3, 6); hold on; grid on;
plot([0, 0.3, 0.3, t_max], [0, 0, 0.7, 0.7], 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
for k = 1:length(noise_vec)
    plot(hist_noise_t{k}, hist_noise_P{k}, 'Color', colors(k,:), 'DisplayName', sprintf('Noise = %g', noise_vec(k)));
end
title('Pole Placement: Noise Variation'); xlabel('Time (s)'); ylabel('Velocity'); legend('Location','best');


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