function min_snr = getMinSNRDecimal(phone_number, L, improved)
% GETMINSNR Finds the minimum SNR for robust DTMF decoding (Decimal Precision)
% 
% Usage: min_snr = getMinSNR('01205978436', L_min, @dtmfdesign)

    if nargin < 3
        design_func = @dtmfdesign;
    end
    
    fs = 8000;
    num_trials = 20;
    
    % --- STAGE 1: COARSE INTEGER SEARCH ---
    current_snr = 30; % Start high
    passed_integer_snr = current_snr;
    
    while true
        successes = 0;
        for test = 1:num_trials
            xx = dtmfdial(phone_number, current_snr);
            decoded = dtmfrun(xx, L, fs, design_func);
            if strcmp(decoded, phone_number)
                successes = successes + 1;
            end
        end
        
        success_rate = (successes / num_trials) * 100;
        
        if success_rate >= 90
            passed_integer_snr = current_snr; % Keep track of last good integer
            current_snr = current_snr - 1; 
        else
            % Failed at this integer.
            break; 
        end
        
        if current_snr < -20, break; end
    end

    % --- STAGE 2: FINE DECIMAL SEARCH ---
    % We start at the last passing integer and go down by 0.1
    current_snr = passed_integer_snr - 0.1;
    min_snr = passed_integer_snr; % Default to the last known good integer
    
    while true
        successes = 0;
        for test = 1:num_trials
            xx = dtmfdial(phone_number, current_snr);
            decoded = dtmfrun(xx, L, fs, design_func);
            if strcmp(decoded, phone_number)
                successes = successes + 1;
            end
        end
        
        success_rate = (successes / num_trials) * 100;
        
        if success_rate >= 90
            min_snr = current_snr;
            current_snr = current_snr - 0.1;
        else
            % Failed at this decimal. The previous iteration was our true min.
            break;
        end
        
        if current_snr < (passed_integer_snr - 1.1), break; end
    end
    
    min_snr = round(min_snr, 1);
end