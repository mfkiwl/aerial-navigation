% quadrotor paramters
qr_params = struct('M', 1, 'm', 1, 'L', 1, 'l', 1, 'g', 9.81);

tspan = [0, 10];
initial_conditions = zeros(16, 1);
[t, y] = ode45(@(t,y)quadrotor_dynamics(t, y, qr_params), tspan, initial_conditions);
