function [u1, u2, u3, u4] = controller(state, params)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

m = params.m;
M = params.M;
l = params.l;
L = params.L;
g = params.g;

u0 = sqrt(1/4 * (M + 4 * m) * g);

u1 = u0;
u2 = -u0;
u3 = u0;
u4 = -u0;

end

