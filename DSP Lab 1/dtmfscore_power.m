function score = dtmfscore_power(xx, hh)
% DTMFSCORE_POWER
% Returns the RMS power of the filtered output.

    % Normalize input
    xx = xx*(2/max(abs(xx)));
    
    % Filter the signal
    yy = conv(xx, hh);
    
    % Return the RMS power
    score = sqrt(mean(yy.^2));
end