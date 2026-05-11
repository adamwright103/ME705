function hh = dtmfdesignEllip(fcent, L, fs)
%DTMFDESIGNELLIP
%     hh = dtmfdesignEllip(fcent, L, fs)
%       returns a matrix where each row is the truncated 
%       impulse response of a stable IIR Elliptic BPF.
%==========================================

hh = zeros(length(fcent), L + 1);
N = 4096;

% Define bandwidth and filter characteristics
BW = 80;    % 100 Hz bandwidth (+/- 50 Hz around center)
Rp = 3;      % 1 dB peak-to-peak ripple in the passband
Rs = 45;     % 45 dB of attenuation in the stopband
order = 4;

for i = 1:length(fcent)
    % 1. Define the normalized cutoff frequencies (0 to 1, where 1 is Nyquist)
    f_low = (fcent(i) - BW/2) / (fs/2);
    f_high = (fcent(i) + BW/2) / (fs/2);
    
    % 2. Design the stable Elliptic Bandpass Filter
    [b, a] = ellip(order, Rp, Rs, [f_low, f_high], 'bandpass');
    
    % --- STABILITY CHECK ---
    % Find the poles (roots of the denominator 'a')
    poles = roots(a);
    max_pole_magnitude = max(abs(poles));
    
    if max_pole_magnitude >= 1
        % This will likely never trigger in MATLAB, but it guarantees safety
        error('Unstable filter generated at %d Hz! Max pole magnitude: %f', fcent(i), max_pole_magnitude);
    end
    % -----------------------
    
    % 3. Extract the Impulse Response of the IIR filter to length L+1
    % (Transposed to ensure it is a row vector)
    h_ellip = impz(b, a, L + 1)';
    
    % 4. Scale it so the maximum magnitude response is exactly 1
    [H, ~] = freqz(h_ellip, 1, N);
    peak_val = max(abs(H));
    beta = 1 / peak_val;
    
    hh(i, :) = beta * h_ellip;
end
end