function [t, xs] = euler_integration(fun, t0, t1, ts, x0, varargin)

if ~isempty(varargin)
    de_stiffing = varargin{1};
    xd_max = varargin{2};
else
    de_stiffing = 0;
end


Nv = length(x0) / 2

t = [t0:ts:t1]';
xs = zeros(length(t), 2*Nv);
xt = x0;
for ii = 1:length(t)
    if rem(ii, 1000) == 0
        disp(['STEP ', num2str(ii), ' out of ', num2str(length(t))]);
    end
    tt = t(ii);
    xd = fun(tt, xt);
    if de_stiffing
        for jj = 1:Nv
            if abs(xd(jj)) > xd_max(jj)
                xd(jj) = sign(xd(jj))*xd_max(jj);
            end
        end
    end
    xt = xt + xd.*ts;
    xs(ii, :) = xt';
end
    