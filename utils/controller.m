function [u1, u2, u3, u4] = controller(state, params)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

m = params.m;
M = params.M;
l = params.l;
L = params.L;
g = params.g;

u0 = sqrt(1/4 * (M + 4 * m) * g);

K = [
    0.7071   -0.0000    0.5000    0.5000    4.7691   -0.0000    1.0897   -0.0000    0.6546    1.3844    1.0867    0.0000;
    0.0000    0.7071   -0.5000    0.5000    0.0000    4.7691    0.0000    1.0897   -0.6546    1.3844    0.0000    1.0867;
   -0.7071   -0.0000    0.5000    0.5000   -4.7691    0.0000   -1.0897    0.0000    0.6546    1.3844   -1.0867    0.0000;
   -0.0000   -0.7071   -0.5000    0.5000   -0.0000   -4.7691   -0.0000   -1.0897   -0.6546    1.3844   -0.0000   -1.0867
];

delta_u = -K * state(1:12);

u1 = u0 + delta_u(1);
u2 = -u0 + delta_u(2);
u3 = u0 + delta_u(3);
u4 = -u0 + delta_u(4);

end

