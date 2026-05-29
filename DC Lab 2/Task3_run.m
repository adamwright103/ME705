clear; clc; close all;
set(0, 'DefaultFigureWindowStyle', 'docked');
[num_Hs, den_Hs] = setupGantryModel(1.5); 

%% 1. Define Common Parameters & Trajectories
Ts = 0.02; % Sample time

% --- Input 1: Step Trajectory (6 seconds) ---
t_step = (0:0.001:6)';
p_step = zeros(size(t_step));
p_step(t_step >= 0 & t_step < 1) = 0.1;
p_step(t_step >= 1 & t_step < 2) = 0.5;
p_step(t_step >= 2 & t_step < 3) = 0.4;
p_step(t_step >= 3 & t_step < 4) = 0.15;
p_step(t_step >= 4 & t_step <= 6)  = 0;

% --- Input 2: Ramp Trajectory (10 seconds, Scaled to meters) ---
t_10s = linspace(0, 10, 1000)';
t_ramp = [0, 2.2, 3.5,  5,  6.2,   8,    9, 10]';
y_ramp = [0, 460, 460,  0,    0, -230, -230,  0]' / 1000; 
p_ramp = interp1(t_ramp, y_ramp, t_10s, 'linear');

% --- Input 3: Smooth Trajectory (10 seconds, Scaled to meters) ---
t_traj = [0, 1.5,   3, 4.5,  6, 7.5,  8.8, 10]';
y_traj = [0, 350, 180, 180, 20, -10, -110,  0]' / 1000;
p_smooth = spline(t_traj, y_traj, t_10s);

% --- Store inputs and configurations for looping ---
inputs_data = {p_step, p_ramp, p_smooth};
inputs_time = {t_step, t_10s, t_10s};
input_names = {'Step', 'Ramp', 'Smooth'};
stop_times  = {'6', '10', '10'};
interp_meth = {'previous', 'linear', 'linear'}; % 'previous' keeps step edges sharp

%% 2. Iterative Tuning (IFT) Setup
num_iterations = 25; 
delta = 1e-4;        
alpha_p = 5000000; alpha_i = 50000; alpha_d = 500; 

% Arrays to store the final tuned parameters
tuned_kp = zeros(1, 3);
tuned_ki = zeros(1, 3);
tuned_kd = zeros(1, 3);

%% 3. Train Controllers
for i = 1:3
    fprintf('\n==================================================\n');
    fprintf('TRAINING CONTROLLER %d ON: %s INPUT\n', i, upper(input_names{i}));
    fprintf('==================================================\n');
    
    % Reset to initial gains for each training session
    kp = 265.8; ki = 0.001; kd = 0.42;
    
    % Set the current training trajectory and stop time
    t_current = inputs_time{i};
    t_stop = stop_times{i};
    Trajectory = timeseries(inputs_data{i}, t_current);
    
    for iter = 1:num_iterations
        % --- Step A: Nominal Simulation ---
        simOut = sim('MotorSlider_Sim_IFT.slx', 'StopTime', t_stop);
        if isa(simOut.error, 'timeseries'); err_nom = simOut.error.Data; else; err_nom = simOut.error; end
        J_nom = mean(err_nom.^2, 'omitnan');
        
        fprintf('Iter %2d | Kp: %6.4f | Ki: %6.4f | Kd: %6.4f | MSE: %.6f\n', iter, kp, ki, kd, J_nom);
            
        % --- Step B: Perturb Kp ---
        kp = kp + delta;
        simOut = sim('MotorSlider_Sim_IFT.slx', 'StopTime', t_stop);
        if isa(simOut.error, 'timeseries'); err_p = simOut.error.Data; else; err_p = simOut.error; end
        J_p = mean(err_p.^2, 'omitnan');
        grad_p = (J_p - J_nom) / delta;
        kp = kp - delta;
        
        % --- Step C: Perturb Ki ---
        ki = ki + delta;
        simOut = sim('MotorSlider_Sim_IFT.slx', 'StopTime', t_stop);
        if isa(simOut.error, 'timeseries'); err_i = simOut.error.Data; else; err_i = simOut.error; end
        J_i = mean(err_i.^2, 'omitnan');
        grad_i = (J_i - J_nom) / delta;
        ki = ki - delta;
        
        % --- Step D: Perturb Kd ---
        kd = kd + delta;
        simOut = sim('MotorSlider_Sim_IFT.slx', 'StopTime', t_stop);
        if isa(simOut.error, 'timeseries'); err_d = simOut.error.Data; else; err_d = simOut.error; end
        J_d = mean(err_d.^2, 'omitnan');
        grad_d = (J_d - J_nom) / delta;
        kd = kd - delta;
        
        % --- Step E: Update Parameters ---
        kp = max(0.001, kp - alpha_p * grad_p);
        ki = max(0.001, ki - alpha_i * grad_i);
        kd = max(0.000, kd - alpha_d * grad_d);
    end
    
    % Save trained parameters
    tuned_kp(i) = kp;
    tuned_ki(i) = ki;
    tuned_kd(i) = kd;
end

%% 4. Cross-Testing Validation
fprintf('\n==================================================\n');
fprintf('TESTING ALL CONTROLLERS ACROSS ALL INPUTS...\n');
fprintf('==================================================\n');

% Storage for results (Rows: Inputs tested on | Columns: Controllers used)
rmse_matrix = zeros(3, 3);
responses = cell(3, 3); 

for input_idx = 1:3
    t_current = inputs_time{input_idx};
    t_stop = stop_times{input_idx};
    Trajectory = timeseries(inputs_data{input_idx}, t_current);
    
    for ctrl_idx = 1:3
        % Load specific controller gains
        kp = tuned_kp(ctrl_idx);
        ki = tuned_ki(ctrl_idx);
        kd = tuned_kd(ctrl_idx);
        
        % Simulate
        simOut = sim('MotorSlider_Sim_IFT.slx', 'StopTime', t_stop);
        
        if isa(simOut.Position, 'timeseries')
            act_time = simOut.Position.Time;
            act_pos  = simOut.Position.Data;
        else
            act_time = simOut.tout;
            act_pos  = simOut.Position;
        end
        
        % Calculate Final RMSE (in mm) using the appropriate interpolation method
        ref_pos_interp = interp1(t_current, inputs_data{input_idx}, act_time, interp_meth{input_idx});
        final_error = (ref_pos_interp - act_pos) * 1000; % Convert error to mm
        rmse_matrix(input_idx, ctrl_idx) = sqrt(mean(final_error.^2, 'omitnan'));
        
        % Store response for plotting
        responses{input_idx, ctrl_idx}.time = act_time;
        responses{input_idx, ctrl_idx}.pos = act_pos;
    end
end

%% 5. Print Performance Table
fprintf('\n------------------------------------------------------------\n');
fprintf(' RMSE PERFORMANCE MATRIX (mm) \n');
fprintf('------------------------------------------------------------\n');
fprintf('%-15s | %-12s | %-12s | %-12s\n', 'Tested On \', 'Ctrl 1 (Step)', 'Ctrl 2 (Ramp)', 'Ctrl 3 (Smooth)');
fprintf('------------------------------------------------------------\n');
for i = 1:3
    fprintf('%-15s | %12.4f | %12.4f | %12.4f\n', ...
        input_names{i}, rmse_matrix(i, 1), rmse_matrix(i, 2), rmse_matrix(i, 3));
end
fprintf('------------------------------------------------------------\n');

%% 6. Plotting the 3x3 Results
figure('Name', 'Controller Cross-Validation', 'Position', [50, 100, 1400, 400]);
colors = {'#D95319', '#EDB120', '#7E2F8E'}; % Unique colors for controllers

for i = 1:3
    subplot(1, 3, i);
    hold on; grid on;
    
    % Plot Reference
    plot(inputs_time{i}, inputs_data{i} * 1000, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
    
    % Plot Controller Responses
    for j = 1:3
        plot(responses{i, j}.time, responses{i, j}.pos * 1000, ...
            'Color', colors{j}, 'LineWidth', 1.2, ...
            'DisplayName', sprintf('Controller %d (%s-trained)', j, input_names{j}));
    end
    
    title(sprintf('Input: %s', input_names{i}), 'FontWeight', 'bold');
    xlabel('Time (seconds)');
    if i == 1
        ylabel('Carriage Position (mm)');
    end
    legend('Location', 'best');
    xlim([0, str2double(stop_times{i})]); % Set x-axis limit dynamically
end