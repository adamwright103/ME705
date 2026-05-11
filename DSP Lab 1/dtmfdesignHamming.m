function  hh = dtmfdesignHamming(fcent, L, fs)
%DTMFDESIGNHAMMING
%     hh = dtmfdesignHamming(fcent, L, fs)
%       returns a matrix where each row is the
%       impulse response of a BPF, one for each frequency
%  fcent = vector of center frequencies
%      L = length of FIR bandpass filters
%     fs = sampling freq  
%==========================================

% Generate a Hamming window (transposed to a row vector)
ww = hamming(L + 1)'; 

hh = zeros(length(fcent), L + 1);
nn = 0:L;
N = 4096;

for i = 1:length(fcent)
    hh_unscaled = cos((2 * pi * fcent(i) / fs) * nn) .* ww;
    [H, ~] = freqz(hh_unscaled, 1, N); 
    peak_val = max(abs(H));
    beta = 1 / peak_val;
    hh(i, :) = beta * hh_unscaled;
end
end