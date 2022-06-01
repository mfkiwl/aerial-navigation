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
    -5.0789   -2.8987    0.5265    4.1243  -71.5807  -38.6746  -10.7105   -5.9042    0.6390    5.2980  -15.7036   -8.4716;
    -4.5813   -4.3798   -0.3054    4.0868  -65.0884  -57.3156   -9.6791   -8.8647   -0.4763    5.2722  -14.5176  -12.2189;
    -5.1388   -2.8989    0.5266    4.1244  -73.3836  -38.6745  -10.8835   -5.9045    0.6390    5.2981  -16.5393   -8.4712;
    -4.5837   -4.4563   -0.3056    4.0869  -65.1139  -59.3360   -9.6834   -9.0713   -0.4764    5.2723  -14.5227  -13.1015
];

delta_u = -K * state(1:12);

u1 = u0 + delta_u(1);
u2 = -u0 + delta_u(2);
u3 = u0 + delta_u(3);
u4 = -u0 + delta_u(4);

end

