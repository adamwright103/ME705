function L_min = getMinOrder(fcent, fs, design_func)
%GETMINORDER Finds the minimum filter length L that satisfies specifications.
%
% Usage: 
%   L_min = getMinOrder(fcent, fs)
%   L_min = getMinOrder(fcent, fs, @dtmfdesignHamming) % Optional custom function
% 
% fcent: array of center frequencies
% fs: sampling frequency

    % If no specific design function is provided, default to dtmfdesign
    if nargin < 3
        design_func = @dtmfdesign;
    end

    % ==========================================
    % a) Coarse Search: Start at L = 20, increase by 10
    % ==========================================
    L_coarse = 20;
    found_coarse = false;

    while ~found_coarse
        specs_met = true;
        
        % Generate filters for current L using the chosen design function
        hh = design_func(fcent, L_coarse, fs);
        
        for i = 1:length(fcent)
            h_i = hh(i, :); 
            
            H_mag = abs(freqz(h_i, 1, fcent, fs));
            
            % 1. Check Passband: Target frequency must be >= 1/sqrt(2)
            if H_mag(i) < 1/sqrt(2)
                specs_met = false;
                break;
            end
            
            % 2. Check Stopband: All other DTMF frequencies must be < 1/4
            for j = 1:length(fcent)
                if i ~= j % If it is NOT the target frequency
                    if H_mag(j) >= 1/4
                        specs_met = false;
                        break;
                    end
                end
            end
            
            if ~specs_met
                break; % Exit filter loop early if this L fails
            end
        end
        
        if specs_met
            found_coarse = true;
        else
            L_coarse = L_coarse + 10;
        end
        
        % Safety net to prevent infinite loops
        if L_coarse > 500
            disp('Error: Could not find a suitable L in coarse search.');
            L_min = -1; % Return an error state
            return;
        end
    end

    % ==========================================
    % b) Fine Search: Start at L_coarse - 10, increase by 1
    % ==========================================
    L_min = 0;
    for L_fine = (L_coarse - 10) : L_coarse
        specs_met = true;
        hh = design_func(fcent, L_fine, fs);
        
        for i = 1:length(fcent)
            h_i = hh(i, :);
            H_mag = abs(freqz(h_i, 1, fcent, fs));
            
            % Check Passband
            if H_mag(i) < 1/sqrt(2)
                specs_met = false;
                break;
            end
            
            % Check Stopband
            for j = 1:length(fcent)
                if i ~= j
                    if H_mag(j) >= 1/4
                        specs_met = false;
                        break;
                    end
                end
            end
            
            if ~specs_met
                break;
            end
        end
        
        if specs_met
            L_min = L_fine;
            break; % We found the absolute minimum L!
        end
    end
end