function [t, xs] = euler_integration(fun, t0, t1, ts, x0)

Nv = length(x0) / 2

t = [t0:ts:t1]';
xs = zeros(length(t), 2*Nv);
xt = x0;
for ii = 1:length(t)
    tt = t(ii);
    xd = fun(tt, xt);
    xt = xt + xd.*ts;
    xs(ii, :) = xt';
end
    