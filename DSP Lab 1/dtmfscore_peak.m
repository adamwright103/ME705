function score = dtmfscore_peak(xx, hh)
% DTMFSCORE_PEAK
% Returns the raw maximum amplitude of the filtered output.

    % Normalize input
    xx = xx * (2 / max(abs(xx))); 
    
    % Filter the signal
    yy = conv(xx, hh);
    
    % Return the raw peak amplitude (Continuous value instead of 0 or 1)
    score = max(abs(yy)); 
end