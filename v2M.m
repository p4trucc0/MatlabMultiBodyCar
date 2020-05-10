function M = v2M(x3)
% If using linear coordinates, transpose.

M = [0      -x3(3)      x3(2);
    x3(3)       0      -x3(1);
    -x3(2)   x3(1)          0];