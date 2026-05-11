function keys = dtmfrunImproved(xx, L, fs)
%DTMFRUN    keys = dtmfrun(xx,L,fs)
%DTMFRUN    keys = dtmfrun(xx,L,fs, @deisgn_func)
%    returns the list of key numbers corresponding
%      to the DTMF waveform, xx.
%            L = filter length
%           fs = sampling freq  
%  design_func = filter design function to use, defualts to dtmfdesign()


freqs = [697,770,852,941,1209,1336,1477,1633];  % list of centre frequencies

hh = dtmfdesign(freqs, L, fs);
%   hh = MATRIX of all the filters. Each row contains the impulse
%        response of one BPF (bandpass filter)

xx_filtered = zeros(size(xx));
for i = 1:length(freqs)
    xx_filtered = xx_filtered + conv(xx, hh(i, :), 'same');
end

dtmf.keys = ...
['1','2','3','A';
'4','5','6','B';
'7','8','9','C';
'*','0','#','D'];
dtmf.colTones = [1209,1336,1477,1633];
dtmf.rowTones = [697;770;852;941];



[nstart,nstop] = dtmfcut(xx_filtered,fs);   %<--Find the start and end points of each tone

%%%% add your lines below to complete the code

keys = blanks(length(nstart)); % Initialize an empty array to store the decoded characters

% Loop through every detected tone segment
for k = 1:length(nstart)
    
    % Extract the short segment for the k-th key press
    xx_seg = xx_filtered(nstart(k):nstop(k));
    
    % Array to keep track of the scores (0 or 1) for all 8 filters
    scores = zeros(1, 8);

        for i = 1:8
            h_i = hh(i, :); 
            scores(i) = dtmfscore_power(xx_seg, h_i);
        end
        
        % Sort the row scores in descending order
        [sorted_rows, row_indices] = sort(scores(1:4), 'descend');
        
        % Sort the column scores in descending order
        [sorted_cols, col_indices] = sort(scores(5:8), 'descend');
        
        row_is_valid = (sorted_rows(1) > sorted_rows(2)) && (sorted_rows(1) > 0.05);
        col_is_valid = (sorted_cols(1) > sorted_cols(2)) && (sorted_cols(1) > 0.05);

        % if k == 1
        %     disp(scores);
        % end
        
        if row_is_valid && col_is_valid
            row_idx = row_indices(1);
            col_idx = col_indices(1);
            
            decoded_key = dtmf.keys(row_idx, col_idx);
            keys(k) = decoded_key;
        else
            % Failed the confidence test (Too much noise, no clear winner)
            keys(k) = '?';
        end
end
end