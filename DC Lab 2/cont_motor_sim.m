% 1. Load the external data first
loaded_data = load('voltageInput.mat');
ts_custom = loaded_data.Voltage; % Extract the timeseries object

% 2. Define parameters for synthetic inputs
t = 0:0.01:5; 
deadzone_volt = 1;
saturation_volt = 24;

% 3. Create synthetic timeseries inputs
% 10V step (steps from 0 to 10 at t = 1s)
v_step_data = 10 * double(t >= 1); 
ts_step = timeseries(v_step_data, t);

% 30V sine wave to trigger the 24V saturation limit
v_sine_data = 30 * sin(2 * pi * 0.2 * t); 
ts_sine = timeseries(v_sine_data, t);

% 4. Store all timeseries inputs and their titles in cell arrays
v_in_list = {ts_custom, ts_step, ts_sine};
input_names = {'Custom Input (voltageInput.mat)', 'Step Input (10V)', 'Sine Wave Input (45V, 0.2Hz)'};

% 5. Loop through each input, simulate, and plot
for i = 1:length(v_in_list)
    % Set the workspace variable 'Voltage' for Simulink to read
    Voltage = v_in_list{i}; 
    
    % Extract raw arrays for plotting the input graph
    t_in = Voltage.Time;
    v_in = squeeze(Voltage.Data); % Squeeze to ensure it's a 1D array
    
    % Dynamically set the simulation stop time based on the input length
    sim_stop_time = num2str(t_in(end));
    
    % Run simulation
    simOut = sim('MotorSlider_Sim', 'StopTime', sim_stop_time);
    
    % Extract the output results
    vel_data = squeeze(simOut.Velocity.Data); 
    vel_time = simOut.Velocity.Time;
    
    % Create a new figure for the current input
    figure('Name', sprintf('Gantry System: %s', input_names{i}), 'Color', 'w');
    
    % Plot Input Voltage
    subplot(2,1,1);
    plot(t_in, v_in, 'b-', 'LineWidth', 1.5);
    hold on;
    yline(saturation_volt, 'r--', sprintf('Saturation Limit (+%gV)', saturation_volt));
    yline(-saturation_volt, 'r--', sprintf('Saturation Limit (-%gV)', saturation_volt));
    yline(deadzone_volt, 'g:', sprintf('Deadzone (+%gV)', deadzone_volt));
    yline(-deadzone_volt, 'g:', sprintf('Deadzone (-%gV)', deadzone_volt));
    title(sprintf('Input Control Effort: %s', input_names{i}));
    xlabel('Time (s)');
    ylabel('Voltage (V)');
    
    % Dynamically set Y-axis limits to show everything cleanly
    y_max = max(abs(v_in));
    ylim([-(y_max + 5), (y_max + 5)]); 
    grid on;
    
    % Plot Output Velocity
    subplot(2,1,2);
    plot(vel_time, vel_data, 'k-', 'LineWidth', 1.5);
    title('Carriage Velocity Output');
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    grid on;
    
    % Add an overall title to the figure
    sgtitle(sprintf('Model Verification: %s', input_names{i}), 'Interpreter', 'none');
end