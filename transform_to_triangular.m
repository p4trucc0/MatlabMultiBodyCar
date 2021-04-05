function out = transform_to_triangular(M_in)
% Input: extended matrix describing a linear system
% Output: same matrix in triangular form.
disp('T IN')
disp(M_in);
n_r = size(M_in, 1);
if n_r == 1
    out = M_in;
else
fc = M_in(:, 1);
zfc = find(fc == 0);
zfc = zfc(end:-1:1);
nfc = find(fc ~= 0);

M_in = M_in([nfc; zfc], :);
if length(nfc) > 1
    for i_r = 2:n_r
        if M_in(i_r, 1) ~= 0
            f_r = M_in(i_r, 1) / M_in(1, 1);
            rwn = M_in(i_r, :) - f_r*M_in(1, :);
            M_in(i_r, :) = rwn;
        end
    end
end

M_i_h = M_in(1, :);
M_i_l = [zeros(n_r - 1, 1) transform_to_triangular(M_in(2:end, 2:end))];
out = [M_i_h; M_i_l];
disp('TR OUT');
disp(out)
end
% keyboard




