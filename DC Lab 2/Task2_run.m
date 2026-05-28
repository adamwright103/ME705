% Pole Placement for PI Controller
clear; clc; close all;

% 1. Define Plant Parameters from P(z) and get continuous model
num_P = 0.0162;
den_P_root = 0.759;
[num_Hs, den_Hs] = setupGantryModel();

% 2. Define Desired Closed-Loop Poles
p1 = 0.75; 
p2 = 0.15;
velocityStep = 0.7;
noise_power = 0;

% 3. Calculate Controller Coefficients (q0, q1)
% Calculate q0
q0 = (1.759 - (p1 + p2)) / num_P;
% Calculate q1
q1 = ((p1 * p2) - den_P_root) / num_P;

% 4. Format for Simulink Workspace
% C(z) = (q0*z + q1) / (z - 1)
num_C = [q0, q1]; % Numerator coefficients [q0, q1]
den_C = [1, -1];  % Denominator coefficients [1, -1] for (z - 1)

%% --- Execution & Analysis ---

% Run the Simulink model
out = sim('MotorSlider_Sim_controlled.slx');

% Extract time and data (assuming timeseries format from To Workspace blocks)
time = out.VelocityDB.Time;
vel_DB = out.VelocityDB.Data;
vel_P = out.VelocityP.Data;

% Create reference signal (step of velocityStep at t = 0.3s)
ref_signal = zeros(size(time));
ref_signal(time >= 0.3) = velocityStep;

% --- Plotting ---
figure('Name', 'Controller Comparison', 'Position', [100, 100, 800, 600]);

% Deadbeat Plot
subplot(2,1,1);
plot(time, ref_signal, 'k--', 'LineWidth', 1.5); hold on;
plot(time, vel_DB, 'b-', 'LineWidth', 1.5);
title('Deadbeat Controller Output vs Reference');
xlabel('Time (s)');
ylabel('Velocity');
legend('Reference', 'Deadbeat', 'Location', 'best');
xlim([0,1]);
grid on;

% Pole Placement Plot
subplot(2,1,2);
plot(time, ref_signal, 'k--', 'LineWidth', 1.5); hold on;
plot(time, vel_P, 'r-', 'LineWidth', 1.5);
title('Pole Placement Controller Output vs Reference');
xlabel('Time (s)');
ylabel('Velocity');
legend('Reference', 'Pole Placement', 'Location', 'best');
xlim([0,1]);
grid on;

% --- Performance Metrics Calculation ---
% Isolate the step response starting at t = 0.3s
step_idx = find(time >= 0.3, 1);
t_step = time(step_idx:end) - 0.3; % Shift time to start at 0
y_DB = vel_DB(step_idx:end);
y_P = vel_P(step_idx:end);

% Calculate metrics manually using the local function
[rt_DB, os_DB, st_DB] = calculate_step_metrics(t_step, y_DB, velocityStep);
[rt_P, os_P, st_P] = calculate_step_metrics(t_step, y_P, velocityStep);

% --- Print Results ---
fprintf('--- Controller Performance Metrics ---\n\n');

fprintf('Deadbeat Controller:\n');
fprintf('  90%% Rise Time : %.4f s\n', rt_DB);
fprintf('  Overshoot     : %.2f %%\n', os_DB);
fprintf('  Settling Time : %.4f s (within 2%%)\n\n', st_DB);

fprintf('Pole Placement Controller:\n');
fprintf('  90%% Rise Time : %.4f s\n', rt_P);
fprintf('  Overshoot     : %.2f %%\n', os_P);
fprintf('  Settling Time : %.4f s (within 2%%)\n', st_P);


%% --- Local Function ---
function [rise_time, overshoot, settling_time] = calculate_step_metrics(t, y, ref)
    % 1. Overshoot %
    peak_val = max(y);
    overshoot = max(0, ((peak_val - ref) / ref) * 100); 

    % 2. 90% Rise Time
    idx_90 = find(y >= 0.9 * ref, 1);
    if ~isempty(idx_90)
        rise_time = t(idx_90);
    else
        rise_time = NaN; % Response never reached 90%
    end

    % 3. Settling Time (2%)
    upper_bound = ref * 1.02;
    lower_bound = ref * 0.98;
    % Find the last index where the signal is OUTSIDE the 2% bounds
    out_of_bounds_idx = find(y > upper_bound | y < lower_bound, 1, 'last');
    
    if isempty(out_of_bounds_idx)
        settling_time = 0; % It was always within bounds
    elseif out_of_bounds_idx == length(y)
        settling_time = NaN; % It never settled within the simulation time
    else
        % The time at the next index is when it finally settled
        settling_time = t(out_of_bounds_idx + 1); 
    end
end