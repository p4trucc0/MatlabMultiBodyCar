function out = solve_linear_system(A, b)
% Finds a solution to A*x = b.

out = zeros(size(b));

n = size(b, 1);

M = [A b];
Mt = transform_to_triangular(M);
% keyboard
for ii = n:-1:1
    if Mt(ii, ii) ~= 0
        out(ii) = (Mt(ii, n+1) - sum(sum(Mt(ii, 1:n).*out'))) / Mt(ii, ii);
    else
        out(ii) = NaN;
    end
end



