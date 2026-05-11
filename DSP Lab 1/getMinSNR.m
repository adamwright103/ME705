function min_snr = getMinSNR(phone_number, L, improved)
% GETMINSNR Finds the minimum SNR for robust DTMF decoding
% 
% Usage: min_snr = getMinSNR('01205978436', L_min, )
%        min_snr = getMinSNR('01205978436', L_min, @dtmfdesignHamming)
%
% This function tests a given phone number against decreasing SNR values.
% It runs 20 trials per SNR and finds the lowest SNR that maintains 
% a 90% or higher success rate (at least 18/20 correct decodings).

    % determines if to use improved dtmfrun
    if nargin < 3
        improved = false;
    end
    
    fs = 8000;
    current_snr = 30; 
    min_snr = current_snr;
    
    while true
        successes = 0;
        num_trials = 20;
        
        for test = 1:num_trials
            % 1. Generate the noisy signal using the provided dtmfdial
            xx = dtmfdial(phone_number, current_snr);
            
            % 2. Decode the signal using your custom dtmfrun function
            if improved
                decoded = dtmfrunImproved(xx, L, fs);
            else
                decoded = dtmfrun(xx, L, fs);
            end
            
            % 3. Check if the decoded string exactly matches the target
            if strcmp(decoded, phone_number)
                successes = successes + 1;
            end
        end
        
        % Calculate success rate for the current SNR
        success_rate = (successes / num_trials) * 100;
        
        % Check if it meets the 90% threshold
        if success_rate >= 90
            % This SNR passed! Save it and try a lower (noisier) one
            min_snr = current_snr;
            current_snr = current_snr - 1; 
        else
            % Failed to meet the 90% threshold.
            break; % Exit the loop
        end
        
        % Safety break to prevent infinite loops if something goes wrong
        if current_snr < -20
            disp('Reached -20 dB without failing. Check filter logic.');
            break;
        end
    end
end