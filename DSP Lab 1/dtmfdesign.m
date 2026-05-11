function  hh = dtmfdesign(fcent, L, fs)
%DTMFDESIGN
%     hh = dtmfdesign(fcent, L, fs)
%       returns a matrix where each column is the
%       impulse response of a BPF, one for each frequency
%  fcent = vector of center frequencies
%      L = length of FIR bandpass filters
%     fs = sampling freq  
%
% The BPFs must be scaled so that the maximum magnitude
% of the frequency response is equal to one.
%==========================================
% [697;770;852;941;1209;1336;1477;1633]; list of centre frequencies

%%%% add your lines below to complete the code

% Rectangular window (maybe try a hamming etc?)
ww = ones(1, L + 1);

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