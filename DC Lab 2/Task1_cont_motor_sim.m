% 1. Load the external data first
clear; clc; close all;
set(0, 'DefaultFigureWindowStyle', 'docked');

loaded_data = load('voltageInput.mat');
ts_custom = loaded_data.Voltage; % Extract the timeseries object

% 2. Define parameters for synthetic inputs
t = 0:0.01:5; 
[num_Hs, den_Hs] = setupGantryModel();

% 3. Create synthetic timeseries inputs
% 10V step (steps from 0 to 10 at t = 0.5s)
v_step_data = 10 * double(t >= 0.5); 
ts_step = timeseries(v_step_data, t);

% 30V sine wave to trigger the 24V saturation limit
v_sine_data = 30 * sin(2 * pi * 0.2 * t); 
ts_sine = timeseries(v_sine_data, t);

% 4. Store all timeseries inputs and their titles in cell arrays
v_in_list = {ts_custom, ts_step, ts_sine};
input_names = {'Custom Input (voltageInput.mat)', 'Step Input (10V)', 'Sine Wave Input (35V, 0.2Hz)'};

% Print a header to the console for the RMSE results
fprintf('\n--- RMSE Simulation Results ---\n');

% 5. Loop through each input, simulate, and plot
for i = 1:length(v_in_list)
    % Set the workspace variable 'Voltage' for Simulink to read
    Voltage = v_in_list{i}; 
    
    % Extract raw arrays for plotting the input graph
    t_in = Voltage.Time;
    v_in = squeeze(Voltage.Data); % Squeeze to ensure it's a 1D array
    
    % Dynamically set the simulation stop time based on the input length
    sim_stop_time = num2str(t_in(end));
    
    % -----------------------------------------------------------------
    % RUN SIMULATION 1: Simplified Model
    % -----------------------------------------------------------------
    simOut_Sim = sim('MotorSlider_Sim', 'StopTime', sim_stop_time);
    vel_data_sim = squeeze(simOut_Sim.Velocity.Data); 
    vel_time_sim = simOut_Sim.Velocity.Time;
    
    % -----------------------------------------------------------------
    % RUN SIMULATION 2: Realistic Model (RS)
    % -----------------------------------------------------------------
    simOut_RS = sim('MotorSlider_RS', 'StopTime', sim_stop_time);
    vel_data_rs = squeeze(simOut_RS.Velocity.Data); 
    vel_time_rs = simOut_RS.Velocity.Time;
    
    % -----------------------------------------------------------------
    % RMSE CALCULATION & CONSOLE PRINT
    % -----------------------------------------------------------------
    % Interpolate RS data to match the SIM time vector
    vel_data_rs_aligned = interp1(vel_time_rs, vel_data_rs, vel_time_sim, 'linear', 'extrap');
    
    % Calculate Root Mean Square Error
    rmse_value = sqrt(mean((vel_data_sim - vel_data_rs_aligned).^2));
    
    % Print the result to the command window
    fprintf('RMSE for %s: %.4f m/s\n', input_names{i}, rmse_value);
    
    % -----------------------------------------------------------------
    % PLOTTING
    % -----------------------------------------------------------------
    % Create a new figure for the current input
    figure('Name', sprintf('Gantry System: %s', input_names{i}), 'Color', 'w');
    
    % Plot Input Voltage
    subplot(2,1,1);
    plot(t_in, v_in, 'b-', 'LineWidth', 1.5);
    hold on;
    
    title(sprintf('Input Control Effort: %s', input_names{i}));
    xlabel('Time (s)');
    ylabel('Voltage (V)');
    
    % Dynamically set Y-axis limits to show everything cleanly
    y_max = max(abs(v_in));
    ylim([-(y_max + 5), (y_max + 5)]); 
    
    % Crop the X-axis for the Step Input graph (which is index 2)
    if i == 2
        xlim([0, 1.5]);
    end
    grid on;
    
    % Plot Output Velocity Comparison
    subplot(2,1,2);
    plot(vel_time_sim, vel_data_sim, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Simplified Model');
    hold on;
    plot(vel_time_rs, vel_data_rs, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Realistic Model (RS)');
    
    % UPDATED: Inject the RMSE value directly into the second subplot's title
    title(sprintf('Carriage Velocity Output Comparison (RMSE = %.4f m/s)', rmse_value));
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    
    % Crop the X-axis for the Step Input graph (which is index 2)
    if i == 2
        xlim([0, 1.5]);
    end
    grid on;
    legend('show', 'Location', 'best');
    hold off;
    
    % Add an overall title to the figure
    sgtitle(sprintf('Model Verification & Comparison: %s', input_names{i}), 'Interpreter', 'none');
end
fprintf('-------------------------------\n\n');