clear; clc; close all;
set(0, 'DefaultFigureWindowStyle', 'docked');

[num_Hs, den_Hs] = setupGantryModel(1.5); 

%% 1. Load Trajectories from .mat Files
Ts = 0.02; % Sample time

% Load files into separate structures to prevent 'traj' variable name collisions
step_file   = load('steps.mat');
ramp_file   = load('ramps.mat');
smooth_file = load('trajectory.mat');

% Extract and flatten data/time vectors from the timeseries objects
t_step   = step_file.traj.Time;
p_step   = squeeze(step_file.traj.Data);

t_ramp   = ramp_file.traj.Time;
p_ramp   = squeeze(ramp_file.traj.Data);

t_smooth = smooth_file.traj.Time;
p_smooth = squeeze(smooth_file.traj.Data);

% --- Store inputs and configurations for looping ---
inputs_data = {p_step / 1000, p_ramp / 1000, p_smooth / 1000};
inputs_time = {t_step, t_ramp, t_smooth};
input_names = {'Step', 'Ramp', 'Trajectory'};

% Dynamically set stop times based on the final time entry of each file
stop_times  = {num2str(t_step(end)), num2str(t_ramp(end)), num2str(t_smooth(end))};
interp_meth = {'previous', 'linear', 'linear'}; % 'previous' keeps step edges sharp

%% 2. Iterative Tuning (IFT) Setup
num_iterations = 3000; % Change this to 500 once you confirm stability!

% --- FIX B: Safety Guardrails (Maximum allowed gain change per iteration) ---
max_step_kp = 5.0;   % Kp cannot jump by more than 5.0 in one iteration
max_step_ki = 0.05;  % Ki cannot jump by more than 0.05 in one iteration
max_step_kd = 0.02;  % Kd cannot jump by more than 0.02 in one iteration

% Tuning Learning Rates (Alphas)
% Note: Because relative deltas alter gradient scales, you may want to play with these.
alpha_p = 500000;     
alpha_i = 15000;     % Slightly bumped up to help Ki wake up
alpha_d = 300;   

% Arrays to store the final tuned parameters
tuned_kp = zeros(1, 3);
tuned_ki = zeros(1, 3);
tuned_kd = zeros(1, 3);

% Arrays to store historical data for iteration plotting
history_rmse = zeros(3, num_iterations);
history_kp = zeros(3, num_iterations);
history_ki = zeros(3, num_iterations);
history_kd = zeros(3, num_iterations);

%% 3. Train Controllers
for i = 1:3
    fprintf('\n==================================================\n');
    fprintf('TRAINING CONTROLLER %d ON: %s INPUT\n', i, upper(input_names{i}));
    fprintf('==================================================\n');
    
    % Reset to initial gains for each training session
    kp = 265.8; ki = 0.001; kd = 0.1;
    
    % Set the current training trajectory and stop time
    t_current = inputs_time{i};
    t_stop = stop_times{i};
    Trajectory = timeseries(inputs_data{i}, t_current);
    
    for iter = 1:num_iterations
        % --- Step A: Nominal Simulation ---
        simOut = sim('MotorSlider_Sim_IFT.slx', 'StopTime', t_stop);
        if isa(simOut.error, 'timeseries')
            err_nom = simOut.error.Data; 
            t_arr = simOut.error.Time;
        else
            err_nom = simOut.error; 
            t_arr = simOut.tout;
        end
        
        % --- FIX C: Dynamic Multi-Window Masking for Step Discontinuities ---
        if i == 1
            step_edges = [0, 1, 2, 3, 4]; % Exact times your steps occur
            W = 0.12;                     % Masking window: 60ms (3 sample periods)
            mask = true(size(t_arr));
            for e = step_edges
                % Set mask to false only during the immediate transient window
                mask = mask & ~(t_arr >= e & t_arr < (e + W));
            end
            J_nom = mean(err_nom(mask).^2, 'omitnan');
        else
            J_nom = mean(err_nom.^2, 'omitnan');
        end
        
        % Store history for plots (convert MSE to RMSE in mm)
        history_rmse(i, iter) = sqrt(J_nom) * 1000; 
        history_kp(i, iter) = kp;
        history_ki(i, iter) = ki;
        history_kd(i, iter) = kd;
        
        fprintf('Iter %2d | Kp: %6.4f | Ki: %6.4f | Kd: %6.4f | MSE: %.6f\n', iter, kp, ki, kd, J_nom);
            
        % --- Step B: Perturb Kp (FIX A: Relative Delta) ---
        delta_p = max(1e-4, 0.005 * kp); 
        kp = kp + delta_p;
        simOut = sim('MotorSlider_Sim_IFT.slx', 'StopTime', t_stop);
        if isa(simOut.error, 'timeseries'); err_p = simOut.error.Data; t_arr = simOut.error.Time; else; err_p = simOut.error; t_arr = simOut.tout; end
        
        if i == 1
            J_p = mean(err_p(mask).^2, 'omitnan'); % Reuses the exact same step mask
        else
            J_p = mean(err_p.^2, 'omitnan');
        end
        grad_p = (J_p - J_nom) / delta_p;
        kp = kp - delta_p;
        
        % --- Step C: Perturb Ki (FIX A: Relative Delta) ---
        delta_i = max(1e-4, 0.01 * ki); 
        ki = ki + delta_i;
        simOut = sim('MotorSlider_Sim_IFT.slx', 'StopTime', t_stop);
        if isa(simOut.error, 'timeseries'); err_i = simOut.error.Data; t_arr = simOut.error.Time; else; err_i = simOut.error; t_arr = simOut.tout; end
        
        if i == 1
            J_i = mean(err_i(mask).^2, 'omitnan');
        else
            J_i = mean(err_i.^2, 'omitnan');
        end
        grad_i = (J_i - J_nom) / delta_i;
        ki = ki - delta_i;
        
        % --- Step D: Perturb Kd (FIX A: Relative Delta) ---
        delta_d = max(1e-4, 0.01 * kd); 
        kd = kd + delta_d;
        simOut = sim('MotorSlider_Sim_IFT.slx', 'StopTime', t_stop);
        if isa(simOut.error, 'timeseries'); err_d = simOut.error.Data; t_arr = simOut.error.Time; else; err_d = simOut.error; t_arr = simOut.tout; end
        
        if i == 1
            J_d = mean(err_d(mask).^2, 'omitnan');
        else
            J_d = mean(err_d.^2, 'omitnan');
        end
        grad_d = (J_d - J_nom) / delta_d;
        kd = kd - delta_d;
        
        % --- Step E: Update Parameters (FIX B: Gradient Clipping) ---
        step_p = alpha_p * grad_p;
        step_i = alpha_i * grad_i;
        step_d = alpha_d * grad_d;
        
        % Clip updates to protect against erratic cost landscape cliffs
        step_p = sign(step_p) * min(abs(step_p), max_step_kp);
        step_i = sign(step_i) * min(abs(step_i), max_step_ki);
        step_d = sign(step_d) * min(abs(step_d), max_step_kd);
        
        % Apply safe updates
        kp = max(0.001, kp - step_p);
        ki = max(0.001, ki - step_i);
        kd = max(0.000, kd - step_d);
    end
    
    % Save tuned parameters
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
        final_error = (ref_pos_interp - act_pos); % Both are now beautifully in meters
        
        % Multiply by 1000 here so the printed table displays in mm
        rmse_matrix(input_idx, ctrl_idx) = sqrt(mean(final_error.^2, 'omitnan')) * 1000;

        % Store response for plotting
        responses{input_idx, ctrl_idx}.time = act_time;
        responses{input_idx, ctrl_idx}.pos = act_pos;
        responses{input_idx, ctrl_idx}.error = final_error; 
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

% Print Final Tuned PID Gains for each controller
fprintf('%-15s | %12.4f | %12.4f | %12.4f\n', 'Final Kp', tuned_kp(1), tuned_kp(2), tuned_kp(3));
fprintf('%-15s | %12.4f | %12.4f | %12.4f\n', 'Final Ki', tuned_ki(1), tuned_ki(2), tuned_ki(3));
fprintf('%-15s | %12.4f | %12.4f | %12.4f\n', 'Final Kd', tuned_kd(1), tuned_kd(2), tuned_kd(3));

fprintf('------------------------------------------------------------\n');

%% 6. Figure 1: Controller Cross-Validation (Responses & Errors)
figure('Name', 'Controller Cross-Validation');
colors = {'#D95319', '#EDB120', '#7E2F8E'}; % Unique colors for controllers
for i = 1:3
    % --- Top Row: Position Responses ---
    subplot(2, 3, i);
    hold on; grid on;
    
    % Plot Reference
    plot(inputs_time{i}, inputs_data{i} * 1000, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
    
    % Plot Controller Responses
    for j = 1:3
        plot(responses{i, j}.time, responses{i, j}.pos * 1000, ...
            'Color', colors{j}, 'LineWidth', 1.2, ...
            'DisplayName', sprintf('Ctrl %d (%s-trained)', j, input_names{j}));
    end
    
    title(sprintf('Input: %s', input_names{i}), 'FontWeight', 'bold');
    xlabel('Time (s)');
    if i == 1
        ylabel('Carriage Position (mm)');
    end
    legend('Location', 'best');
    xlim([0, str2double(stop_times{i})]); 
    
    % --- Bottom Row: Errors Over Time ---
    subplot(2, 3, i + 3);
    hold on; grid on;
    
    for j = 1:3
        plot(responses{i, j}.time, responses{i, j}.error * 1000, ... 
            'Color', colors{j}, 'LineWidth', 1.2, ...
            'DisplayName', sprintf('Ctrl %d Error', j));
    end
    
    title(sprintf('%s Error', input_names{i}), 'FontWeight', 'bold');
    xlabel('Time (s)');
    if i == 1
        ylabel('Error (mm)');
    end
    legend('Location', 'best');
    xlim([0, str2double(stop_times{i})]);
end

%% 7. Figure 2: IFT Process (RMSE vs Iteration)
figure('Name', 'IFT Convergence');
for i = 1:3
    subplot(1, 3, i); % Creates a 1-row, 3-column layout
    
    plot(1:num_iterations, history_rmse(i, :), 'Color', colors{i}, ...
        'LineWidth', 2, 'Marker', 'o', 'DisplayName', sprintf('%s Training', input_names{i}));
    
    grid on;
    title(sprintf('%s Controller Convergence', input_names{i}), 'FontWeight', 'bold');
    xlabel('Iteration Number');
    ylabel('RMSE (mm)');
    legend('Location', 'best');
end

%% 8. Figure 3: Controller Gains vs Iteration
figure('Name', 'Controller Gains Evolution');
gain_names = {'Kp', 'Ki', 'Kd'};
history_gains = {history_kp, history_ki, history_kd};
for g = 1:3
    subplot(1, 3, g);
    hold on; grid on;
    
    for i = 1:3
        plot(1:num_iterations, history_gains{g}(i, :), 'Color', colors{i}, ...
            'LineWidth', 1.5, 'Marker', '.', 'DisplayName', sprintf('%s Training', input_names{i}));
    end
    
    title(sprintf('%s Gain vs Iteration', gain_names{g}), 'FontWeight', 'bold');
    xlabel('Iteration Number');
    ylabel('Gain Value');
    legend('Location', 'best');
end