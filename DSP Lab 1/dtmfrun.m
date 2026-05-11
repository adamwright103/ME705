function keys = dtmfrun(xx, L, fs)
%DTMFRUN    keys = dtmfrun(xx,L,fs)
%DTMFRUN    keys = dtmfrun(xx,L,fs, @deisgn_func)
%    returns the list of key numbers corresponding
%      to the DTMF waveform, xx.
%            L = filter length
%           fs = sampling freq  


freqs = [697,770,852,941,1209,1336,1477,1633];  % list of centre frequencies

hh = dtmfdesign(freqs, L, fs);
%   hh = MATRIX of all the filters. Each row contains the impulse
%        response of one BPF (bandpass filter)

dtmf.keys = ...
['1','2','3','A';
'4','5','6','B';
'7','8','9','C';
'*','0','#','D'];
dtmf.colTones = [1209,1336,1477,1633];
dtmf.rowTones = [697;770;852;941];

[nstart,nstop] = dtmfcut(xx,fs);   %<--Find the start and end points of each tone

%%%% add your lines below to complete the code

keys = blanks(length(nstart)); % Initialize an empty array to store the decoded characters

% Loop through every detected tone segment
for k = 1:length(nstart)
    
    % Extract the short segment for the k-th key press
    xx_seg = xx(nstart(k):nstop(k));
    
    % Array to keep track of the scores (0 or 1) for all 8 filters
    scores = zeros(1, 8);
    
        % Test this segment against all 8 bandpass filters
        for i = 1:8
            % Note: The comments specify that hh has filters in COLUMNS.
            % If your dtmfdesign function was altered to store them in rows, 
            % you would change this to hh(i, :) 
            h_i = hh(i, :); 
        
            % Calculate the score using your dtmfscore function
            scores(i) = dtmfscore(xx_seg, h_i);
        end
    
        % The first 4 frequencies (indices 1 to 4) represent the rows
        row_idx = find(scores(1:4) == 1);
    
        % The last 4 frequencies (indices 5 to 8) represent the columns
        col_idx = find(scores(5:8) == 1);

        if isscalar(row_idx) && isscalar(col_idx)
            % Map the indices to the keypad matrix and append to our keys list
            decoded_key = dtmf.keys(row_idx, col_idx);
            keys(k) = decoded_key;
        else
            % If noise causes a false detection (0 or 2+ tones in a group),
            % append a '?' to indicate an error for this segment.
            keys(k) = '?';
        end
end
end